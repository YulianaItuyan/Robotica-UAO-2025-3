#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ====== PCA9685 ======
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
#define SERVOMIN 150
#define SERVOMAX 600

// Brazo 1 (derecho) = A
#define SERVO1_BRAZO1  0
#define SERVO2_BRAZO1  1
#define SERVO3_BRAZO1  2
// Brazo 2 (izquierdo) = B
#define SERVO1_BRAZO2  3
#define SERVO2_BRAZO2  6
#define SERVO3_BRAZO2  5

// ====== I2C AS5600 (0x36) + TCA9548A (0x70) ======
#define AS5600_ADDR 0x36
#define USE_TCA9548A 1
#define TCA_ADDR 0x70

int magnetStatus = 0;
const uint8_t ENC_BRAZO1_CH[3] = {0, 1, 2};
const uint8_t ENC_BRAZO2_CH[3] = {3, 4, 5};

// ====== Estado general ======
int brazoActivo = 1; // 1=derecho(A), 2=izquierdo(B)
String buf;

// ====== Tare/lecturas ======
float startAngle[6] = {0}; // 0..2 brazo1, 3..5 brazo2
float lastDeg[6]    = {0};
float lastCorr[6]   = {0};

// ====== Control ======
volatile float targetA[3] = {90,90,0};    // ref brazo A (articular)
volatile float targetB[3] = {90,90,180};  // ref brazo B (articular)
float KpA[3] = { 1.0, 1.0, 1.0 };
float KpB[3] = { 1.0, 1.0, 1.0 };
const float tolDeg[3] = { 0.5, 0.5, 0.5 };

const uint16_t CTRL_DT_MS = 20;
unsigned long lastCtrl = 0;

// ====== CRC-8 Dallas/Maxim (poly 0x31, init 0x00) ======
uint8_t crc8_maxim(const String& s) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < s.length(); ++i) {
    crc ^= (uint8_t)s[i];
    for (uint8_t b = 0; b < 8; ++b) {
      if (crc & 0x80) crc = (uint8_t)((crc << 1) ^ 0x31);
      else            crc <<= 1;
    }
  }
  return crc;
}

void sendLine(const String& payload) {
  uint8_t cs = crc8_maxim(payload);
  char suff[4];
  sprintf(suff, "*%02X", cs);
  Serial.print(payload);
  Serial.print(suff);
  Serial.print('\n');
}

bool verifyChecksum(const String& line, String& payload_out) {
  int star = line.lastIndexOf('*');
  if (star < 0 || (line.length() - star) != 3) return false; // "*XX"
  payload_out = line.substring(0, star);
  auto hex2n = [](char c)->int {
    if (c>='0'&&c<='9') return c-'0';
    if (c>='A'&&c<='F') return 10 + (c-'A');
    if (c>='a'&&c<='f') return 10 + (c-'a');
    return -1;
  };
  int h = hex2n(line[star+1]);
  int l = hex2n(line[star+2]);
  if (h<0 || l<0) return false;
  uint8_t rx = (uint8_t)((h<<4)|l);
  uint8_t calc = crc8_maxim(payload_out);
  return rx == calc;
}

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
  while ((magnetStatus & 32) != 32) {
    magnetStatus = 0;
    Wire.beginTransmission(0x36);
    Wire.write(0x0B);
    Wire.endTransmission();
    Wire.requestFrom(0x36, 1);
    while (Wire.available() == 0);
    magnetStatus = Wire.read();
    Serial.println("❌ Magneto no detectado, ajuste la posición.");
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

// ---------- LIMITES ----------
float limitServoA(int j, float cmd) {
  if (j == 0) { if (cmd > 180) cmd = 180; }
  if (j == 1) { if (cmd > 90)  cmd = 90;  }
  if (j == 2) { if (cmd > 90) cmd = 90; }
  if (cmd < 0) cmd = 0;
  return cmd;
}
float limitServoB(int j, float cmd) {
  if (j == 0) { if (cmd > 105) cmd = 105; }
  if (j == 1) { if (cmd > 90) cmd = 90; }
  if (j == 2) { if (cmd > 90) cmd = 90; }
  if (cmd < 0) cmd = 0;
  return cmd;
}

// ---------- Medida encoder -> articular ----------
float measToJointA(int j, float enc_deg) {
  if (enc_deg < 0) enc_deg += 360;
  if (enc_deg > 180) enc_deg = fmod(enc_deg, 360.0f);
  if (enc_deg > 180) enc_deg = 360.0f - enc_deg;
  return enc_deg;
}
float measToJointB(int j, float enc_deg) {
  if (j == 1) {
    float t = 180.0f - enc_deg;
    if (t < 0) t += 360.0f;
    if (t < 0) t = 0;
    if (t > 180) t = 180;
    return t;
  }
  if (enc_deg < 0) enc_deg += 360;
  if (enc_deg > 180) enc_deg = fmod(enc_deg, 360.0f);
  if (enc_deg > 180) enc_deg = 360.0f - enc_deg;
  return enc_deg;
}

String f1(float v) { return String(v, 1); }

// ---------- Control + OKA/OKB SOLO con medidos ----------
void controlStep() {
  unsigned long now = millis();
  if (now - lastCtrl < CTRL_DT_MS) return;
  lastCtrl = now;

  const uint8_t REACH_HITS = 3;
  static uint8_t hitA[3] = {0,0,0}, hitB[3] = {0,0,0};
  bool allA=false, allB=false;

  // === Brazo A (derecho) ===
  float encA_raw[3];
  for (uint8_t j = 0; j < 3; ++j) encA_raw[j] = readCorrectedFor(ENC_BRAZO1_CH[j], j);
  float measA[3];
  for (uint8_t j = 0; j < 3; ++j) {
    float measJoint = measToJointA(j, encA_raw[j]); measA[j] = measJoint;
    float ref = targetA[j];
    if (ref < 0) ref = 0; if (ref > 180) ref = 180;
    float err = ref - measJoint;
    float cmd = measJoint + KpA[j]*err;
    cmd = limitServoA(j, cmd);
    uint8_t ch = (j==0)? SERVO1_BRAZO1 : (j==1? SERVO2_BRAZO1 : SERVO3_BRAZO1);
    servoWrite(ch, cmd);

    float aerr = (err < 0)? -err : err;
    if (aerr <= tolDeg[j]) { if (hitA[j] < 255) hitA[j]++; } else { hitA[j] = 0; }
  }
  allA = (hitA[0] >= REACH_HITS) && (hitA[1] >= REACH_HITS) && (hitA[2] >= REACH_HITS);

  // === Brazo B (izquierdo) ===
  float encB_raw[3];
  for (uint8_t j = 0; j < 3; ++j) encB_raw[j] = readCorrectedFor(ENC_BRAZO2_CH[j], 3 + j);
  float measB[3];
  for (uint8_t j = 0; j < 3; ++j) {
    float measJoint = measToJointB(j, encB_raw[j]); measB[j] = measJoint;
    float ref = targetB[j];
    if (ref < 0) ref = 0; if (ref > 180) ref = 180;
    float err = ref - measJoint;
    float cmd = ref + KpB[j]*err;
    cmd = limitServoB(j, cmd);
    uint8_t ch = (j==0)? SERVO1_BRAZO2 : (j==1? SERVO2_BRAZO2 : SERVO3_BRAZO2);
    servoWrite(ch, cmd);

    float aerr = (err < 0)? -err : err;
    if (aerr <= tolDeg[j]) { if (hitB[j] < 255) hitB[j]++; } else { hitB[j] = 0; }
  }
  allB = (hitB[0] >= REACH_HITS) && (hitB[1] >= REACH_HITS) && (hitB[2] >= REACH_HITS);

  // --- Enviar SOLO medidos cuando llegó (OKA/OKB) ---
  static bool sentA=false, sentB=false;
  if (allA && !sentA) {
    String p = "OKA<" + f1(measA[0]) + "," + f1(measA[1]) + "," + f1(measA[2]) + ">";
    sendLine(p);
    sentA = true;
  }
  if (!allA) sentA = false;

  if (allB && !sentB) {
    String p = "OKB<" + f1(measB[0]) + "," + f1(measB[1]) + "," + f1(measB[2]) + ">";
    sendLine(p);
    sentB = true;
  }
  if (!allB) sentB = false;
}

// ---------- Setup Mejorado ----------
void verifyMagnetAndTare() {
  Serial.println("🔧 Verificando magnetos y aplicando tare con offset...");
  const float desiredA[3] = {90.0f, 90.0f, 0.0f};
  const float desiredB[3] = {90.0f, 90.0f, 180.0f};

  for (uint8_t j = 0; j < 3; j++) {
    tcaSelect(ENC_BRAZO1_CH[j]);
    checkMagnetPresence();
    float currentPos = as5600ReadDeg();
    startAngle[j] = currentPos - desiredA[j];
    if (startAngle[j] < 0) startAngle[j] += 360.0f;
    Serial.print("A"); Serial.print(j+1);
    Serial.print(" startAngle = "); Serial.println(startAngle[j], 2);
  }

  for (uint8_t j = 0; j < 3; j++) {
    tcaSelect(ENC_BRAZO2_CH[j]);
    checkMagnetPresence();
    float currentPos = as5600ReadDeg();
    startAngle[3 + j] = currentPos - desiredB[j];
    if (startAngle[3 + j] < 0) startAngle[3 + j] += 360.0f;
    Serial.print("B"); Serial.print(j+1);
    Serial.print(" startAngle = "); Serial.println(startAngle[3 + j], 2);
  }

  Serial.println("✅ Tare con offset aplicado");
  Serial.println("📊 Resumen de posiciones de tare:");
  Serial.print("  Brazo 1: ["); 
  Serial.print(startAngle[0], 1); Serial.print("°, ");
  Serial.print(startAngle[1], 1); Serial.print("°, ");
  Serial.print(startAngle[2], 1); Serial.println("°]");
  Serial.print("  Brazo 2: [");
  Serial.print(startAngle[3], 1); Serial.print("°, ");
  Serial.print(startAngle[4], 1); Serial.print("°, ");
  Serial.print(startAngle[5], 1); Serial.println("°]");
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("🚀 Iniciando sistema de brazos robóticos...");

  Wire.begin();
  Wire.setClock(400000L);
  Serial.println("✓ I2C inicializado a 400kHz");

  pwm.begin();
  pwm.setPWMFreq(60);
  Serial.println("✓ PCA9685 inicializado");

  targetA[0]=90; targetA[1]=90; targetA[2]=0;
  targetB[0]=90; targetB[1]=90; targetB[2]=180;

  Serial.println("🎯 Moviendo servos a posición inicial...");
  servoWrite(SERVO1_BRAZO1, 90);
  servoWrite(SERVO2_BRAZO1, 90);
  servoWrite(SERVO3_BRAZO1, 0);
  servoWrite(SERVO1_BRAZO2, 90);
  servoWrite(SERVO2_BRAZO2, 90);
  servoWrite(SERVO3_BRAZO2, 180);

  Serial.println("⏳ Esperando estabilización de servos (3 segundos)...");
  delay(3000);

  verifyMagnetAndTare();

  Serial.println("🎉 Sistema listo");
  Serial.println("📋 Comandos (con CRC):");
  Serial.println("  G<2|3>*CS  → Cambia brazo activo");
  Serial.println("  <a,b,c>*CS → Actualiza objetivos del brazo activo");
  Serial.println("📡 Respuestas (con CRC): ACKA/B<applied>, OKA/B<meas1,meas2,meas3>");
}

// ---------- Parser de payload (sin *CS) ----------
void parseAndApply(const String& s) {
  if (s.startsWith("G<")) {
    int l = s.indexOf('<');
    int r = s.indexOf('>');
    if (l != -1 && r != -1) {
      int val = s.substring(l + 1, r).toInt();  // 2 o 3
      if (val == 2 || val == 3) {
        brazoActivo = (val == 2) ? 1 : 2;
        // (Ya no enviamos ENC aquí: solo medidos al llegar)
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

  if (v1 < 0) v1 = 0; if (v1 > 180) v1 = 180;
  if (v2 < 0) v2 = 0; if (v2 > 180) v2 = 180;
  if (v3 < 0) v3 = 0; if (v3 > 180) v3 = 180;

  if (brazoActivo == 1) {
    targetA[0] = v1; targetA[1] = v2; targetA[2] = v3;
    sendLine("ACKA<applied>");
  } else {
    targetB[0] = v1; targetB[1] = v2; targetB[2] = v3;
    sendLine("ACKB<applied>");
  }
}

// ---------- Loop principal ----------
void loop() {
  // Parser de líneas con CRC (no bloqueante)
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n') {
      if (buf.length() > 0) {
        String payload;
        if (!verifyChecksum(buf, payload)) {
          sendLine("ERR<CS>");
          buf = "";
          continue;
        }
        parseAndApply(payload);
        buf = "";
      }
    } else {
      if (buf.length() < 96) buf += ch; else buf = ""; // evita desbordes
    }
  }

  // Control y detección de llegada → OKA/OKB con SOLO medidos
  controlStep();
}
