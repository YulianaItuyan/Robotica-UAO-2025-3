#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ====== PCA9685 ======
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
#define SERVOMIN 150
#define SERVOMAX 600

// Brazo 1 (derecho)
#define SERVO1_BRAZO1  0
#define SERVO2_BRAZO1  1
#define SERVO3_BRAZO1  2
// Brazo 2 (izquierdo)
#define SERVO1_BRAZO2  3
#define SERVO2_BRAZO2  6
#define SERVO3_BRAZO2  5

// ====== I2C AS5600 (0x36) + TCA9548A opcional (0x70) ======
#define AS5600_ADDR 0x36
#define USE_TCA9548A 1
#define TCA_ADDR 0x70

// Canales del TCA para 3 encoders por brazo
const uint8_t ENC_BRAZO1_CH[3] = {0, 1, 2};
const uint8_t ENC_BRAZO2_CH[3] = {3, 4, 5};

// ====== Estado general ======
int brazoActivo = 1; // 1=derecho, 2=izquierdo
String buf;

// ====== AS5600 tare/lecturas ======
float startAngle[6] = {0}; // 0..2 brazo1, 3..5 brazo2
float lastDeg[6]    = {0};
float lastCorr[6]   = {0};

// ====== Control de posición (cerrado) ======
volatile float targetA[3] = {90,90,0};    // objetivos brazo A (articulación)
volatile float targetB[3] = {90,90,180};  // objetivos brazo B (articulación)

// Ganancias P por articulación (deg_servo / deg_error_articular)
float KpA[3] = { 0.5, 0.5, 0.5 };
float KpB[3] = { 0.5, 0.5, 0.5 };

// Tolerancia de cierre (deg de articulación)
const float tolDeg[3] = { 1.5, 1.5, 1.5 };

// Periodo del control (ms)
const uint16_t CTRL_DT_MS = 20;
unsigned long lastCtrl = 0;

// ---------- I2C / TCA ----------
void tcaSelect(uint8_t ch) {
#if USE_TCA9548A
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << (ch & 7));
  Wire.endTransmission();
#else
  (void)ch;
#endif
}

void checkMagnetPresence() {
  int tries = 0;
  while (tries < 200) {
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(0x0B);
    Wire.endTransmission();
    Wire.requestFrom(AS5600_ADDR, 1);
    while (Wire.available() == 0);
    uint8_t st = Wire.read();
    if (st & 0x20) return;
    delay(10);
    tries++;
  }
}

float as5600ReadDeg() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0D);
  Wire.endTransmission();
  Wire.requestFrom(AS5600_ADDR, 1);
  while (Wire.available() == 0);
  uint8_t lowbyte = Wire.read();

  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0C);
  Wire.endTransmission();
  Wire.requestFrom(AS5600_ADDR, 1);
  while (Wire.available() == 0);
  uint8_t highbyte = Wire.read();

  uint16_t raw = ((uint16_t)highbyte << 8) | lowbyte;
  float deg = raw * 0.087890625f; // 360/4096
  if (deg >= 360.0f) deg -= 360.0f;
  return deg;
}

float readCorrectedFor(uint8_t ch, uint8_t encIdx) {
  tcaSelect(ch);
  float d = as5600ReadDeg();
  lastDeg[encIdx] = d;
  float corr = d - startAngle[encIdx];
  if (corr < 0) corr += 360.0f;
  lastCorr[encIdx] = corr;
  return corr;
}

// ---------- Servos ----------
void servoWrite(uint8_t canal, float angulo) {
  if (angulo < 0) angulo = 0;
  if (angulo > 180) angulo = 180;
  int pulseWidth = map((int)angulo, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(canal, 0, pulseWidth);
}

// ---------- MAPEOS (articulación -> ángulo de servo) ----------

// ---------- LIMITES por canal (post-mapeo) ----------
float limitServoA(int j, float cmd) {
  if (j == 0) { if (cmd > 220) cmd = 220; }
  if (j == 1) { if (cmd > 90)  cmd = 90;  }
  if (j == 2) { if (cmd > 180) cmd = 180; } // seguridad
  if (cmd < 0) cmd = 0;
  return cmd;
}

float limitServoB(int j, float cmd) {
  if (j == 0) { if (cmd > 105) cmd = 105; }
  if (j == 1) { if (cmd > 180) cmd = 180; }
  if (j == 2) { if (cmd > 180) cmd = 180; }
  if (cmd < 0) cmd = 0;
  return cmd;
}

// ---------- Convertir medida encoder -> ángulo de articulación esperado ----------
// Nota: ajusta estas funciones si tu encoder no está 1:1 con la articulación.
float measToJointA(int j, float enc_deg) {
  // Por defecto asumimos 1:1 con orientación ya tared.
  // Si tu zero/tara está correcta, esto ya queda en [0..360) y usas 0..180 efectivamente.
  // Recorta a 0..180 por seguridad:
  if (enc_deg < 0) enc_deg += 360;
  if (enc_deg > 180) enc_deg = fmod(enc_deg, 360.0f);
  if (enc_deg > 180) enc_deg = 360.0f - enc_deg; // si excede, pliega (opcional según tu mecánica)
  return enc_deg;
}

float measToJointB(int j, float enc_deg) {
  // Para el eje invertido (j=1) quizá quieras  theta = 180 - enc
  if (j == 1) {
    float t = 180.0f - enc_deg;
    if (t < 0) t += 360.0f;
    if (t < 0) t = 0;
    if (t > 180) t = 180;
    return t;
  }
  // Otros ejes 1:1
  if (enc_deg < 0) enc_deg += 360;
  if (enc_deg > 180) enc_deg = fmod(enc_deg, 360.0f);
  if (enc_deg > 180) enc_deg = 360.0f - enc_deg;
  return enc_deg;
}

// ---------- Control de seguimiento (cerrado) ----------
void controlStep() {
  // Ejecuta cada CTRL_DT_MS
  unsigned long now = millis();dsad
  if (now - lastCtrl < CTRL_DT_MS) return;
  lastCtrl = now;

  // --- filtros de llegada (antirruido) ---
  const uint8_t REACH_HITS = 3;                // N ciclos consecutivos dentro de tolerancia
  static uint8_t hitA[3] = {0,0,0}, hitB[3] = {0,0,0};
  bool allA=false, allB=false;

  // === Brazo 1 (derecho) ===
  float encA[3];
  for (uint8_t j = 0; j < 3; ++j) {
    encA[j] = readCorrectedFor(ENC_BRAZO1_CH[j], j); // idx 0..2
  }
  for (uint8_t j = 0; j < 3; ++j) {
    float measJoint = measToJointA(j, encA[j]);
    float ref = targetA[j];
    if (ref < 0) ref = 0; if (ref > 180) ref = 180;
    float err = ref - measJoint;

    // --- SIN baseMap: comando 1:1 + corrección P ---
    float cmd = ref + KpA[j] * err;

    // Límites de servo
    cmd = limitServoA(j, cmd);

    // Enviar al canal correcto del PCA
    uint8_t ch = (j==0)? SERVO1_BRAZO1 : (j==1? SERVO2_BRAZO1 : SERVO3_BRAZO1);
    servoWrite(ch, cmd);

    // Confirmación de llegada por junta (consecutivos)
    float aerr = (err < 0)? -err : err;
    if (aerr <= tolDeg[j]) { if (hitA[j] < 255) hitA[j]++; } else { hitA[j] = 0; }
  }
  allA = (hitA[0] >= REACH_HITS) && (hitA[1] >= REACH_HITS) && (hitA[2] >= REACH_HITS);

  // === Brazo 2 (izquierdo) ===
  float encB[3];
  for (uint8_t j = 0; j < 3; ++j) {
    encB[j] = readCorrectedFor(ENC_BRAZO2_CH[j], 3 + j); // idx 3..5
  }
  for (uint8_t j = 0; j < 3; ++j) {
    float measJoint = measToJointB(j, encB[j]);
    float ref = targetB[j];
    if (ref < 0) ref = 0; if (ref > 180) ref = 180;
    float err = ref - measJoint;

    // --- SIN baseMap: comando 1:1 + corrección P ---
    float cmd = ref + KpB[j] * err;

    // Límites de servo
    cmd = limitServoB(j, cmd);

    uint8_t ch = (j==0)? SERVO1_BRAZO2 : (j==1? SERVO2_BRAZO2 : SERVO3_BRAZO2);
    servoWrite(ch, cmd);

    // Confirmación de llegada por junta (consecutivos)
    float aerr = (err < 0)? -err : err;
    if (aerr <= tolDeg[j]) { if (hitB[j] < 255) hitB[j]++; } else { hitB[j] = 0; }
  }
  allB = (hitB[0] >= REACH_HITS) && (hitB[1] >= REACH_HITS) && (hitB[2] >= REACH_HITS);

  // --- Mensajes de llegada (una sola vez por meta alcanzada) ---
  // Nota: si spammea, puedes exigir que las 3 cuenten  REACH_HITS+2 para "salir" del estado
  static bool sentA=false, sentB=false;
  if (allA && !sentA) {
    Serial.print("OKA<");
    Serial.print(measToJointA(0, encA[0]), 2); Serial.print(",");
    Serial.print(measToJointA(1, encA[1]), 2); Serial.print(",");
    Serial.print(measToJointA(2, encA[2]), 2); Serial.println(">");
  sentA = true;
}
  if (!allA) sentA = false;

  if (allB && !sentB) {
    Serial.print("OKB<");
    Serial.print(measToJointB(0, encB[0]), 2); Serial.print(",");
    Serial.print(measToJointB(1, encB[1]), 2); Serial.print(",");
    Serial.print(measToJointB(2, encB[2]), 2); Serial.println(">");
  sentB = true;
}
  if (!allB) sentB = false;
}


// ---------- Setup ----------
void setup() {
  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(400000L);

  pwm.begin();
  pwm.setPWMFreq(60);

  // Posición inicial (también como objetivo inicial)
  targetA[0]=90; targetA[1]=90; targetA[2]=0;
  targetB[0]=90; targetB[1]=90; targetB[2]=180;

  servoWrite(SERVO1_BRAZO1, 90);
  servoWrite(SERVO2_BRAZO1, 90);
  servoWrite(SERVO3_BRAZO1, 0);

  servoWrite(SERVO1_BRAZO2, 90);
  servoWrite(SERVO2_BRAZO2, 90);
  servoWrite(SERVO3_BRAZO2, 180);

  // Tare de 6 encoders
  for (uint8_t j = 0; j < 3; ++j) {
    tcaSelect(ENC_BRAZO1_CH[j]); checkMagnetPresence();
    float d = as5600ReadDeg(); delay(5);
    d = 0.5f*(d + as5600ReadDeg());
    startAngle[j] = d;
  }
  for (uint8_t j = 0; j < 3; ++j) {
    tcaSelect(ENC_BRAZO2_CH[j]); checkMagnetPresence();
    float d = as5600ReadDeg(); delay(5);
    d = 0.5f*(d + as5600ReadDeg());
    startAngle[3 + j] = d;
  }

  Serial.println("📡 Listo: PCA9685 + AS5600 con control P cerrado.");
  Serial.println("Comandos: G<2|3> cambia brazo y reporta ENC; <a,b,c> actualiza objetivos del brazo activo.");
}

// Lee 3 encoders del brazo indicado (tared)
void readArmEncoders(int brazo, float out_deg[3]) {
  if (brazo == 1) {
    for (uint8_t j = 0; j < 3; ++j) out_deg[j] = readCorrectedFor(ENC_BRAZO1_CH[j], j);
  } else {
    for (uint8_t j = 0; j < 3; ++j) out_deg[j] = readCorrectedFor(ENC_BRAZO2_CH[j], 3 + j);
  }
}

// ---------- Parser ----------
void parseAndApply(const String& s) {
  if (s.startsWith("G<")) {
    int l = s.indexOf('<');
    int r = s.indexOf('>');
    if (l != -1 && r != -1) {
      int val = s.substring(l + 1, r).toInt();
      if (val == 2 || val == 3) {
        brazoActivo = (val == 2) ? 1 : 2;
        Serial.println(brazoActivo == 1 ?
          "🤖 Cambiado a: Brazo 1 (derecho) - Canales 0,1,2" :
          "🤖 Cambiado a: Brazo 2 (izquierdo) - Canales 3,6,5");

        float enc[3];
        readArmEncoders(brazoActivo, enc);
        Serial.print("ENC<"); Serial.print(val); Serial.print(",");
        Serial.print(enc[0], 2); Serial.print(",");
        Serial.print(enc[1], 2); Serial.print(",");
        Serial.print(enc[2], 2); Serial.println(">");
      }
    }
    return;
  }

  // Movimiento: "<v1,v2,v3>"
  int l = s.indexOf('<');
  int r = s.indexOf('>');
  if (l == -1 || r == -1 || r <= l) return;

  String payload = s.substring(l + 1, r);
  int c1 = payload.indexOf(',');
  int c2 = payload.indexOf(',', c1 + 1);
  if (c1 == -1 || c2 == -1) return;

  float v1 = payload.substring(0, c1).toFloat();
  float v2 = payload.substring(c1 + 1, c2).toFloat();
  float v3 = payload.substring(c2 + 1).toFloat();

  // Recorta a 0..180
  if (v1 < 0) v1 = 0; if (v1 > 180) v1 = 180;
  if (v2 < 0) v2 = 0; if (v2 > 180) v2 = 180;
  if (v3 < 0) v3 = 0; if (v3 > 180) v3 = 180;

  if (brazoActivo == 1) {
    targetA[0] = v1; targetA[1] = v2; targetA[2] = v3;
    Serial.print("🎯 Objetivo A: "); Serial.println(payload);
  } else {
    targetB[0] = v1; targetB[1] = v2; targetB[2] = v3;
    Serial.print("🎯 Objetivo B: "); Serial.println(payload);
  }
}

void loop() {
  // Parser no bloqueante
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n') { parseAndApply(buf); buf = ""; }
    else { if (buf.length() < 64) buf += ch; else buf = ""; }
  }

  // Lazo cerrado
  controlStep();
}
