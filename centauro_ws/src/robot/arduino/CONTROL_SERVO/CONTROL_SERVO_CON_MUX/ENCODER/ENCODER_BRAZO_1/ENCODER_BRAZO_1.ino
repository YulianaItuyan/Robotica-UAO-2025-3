// ——— 3x Servo (PCA9685) + 3x AS5600 (via TCA9548A) + PID(IMC) ———
// Brazo DERECHO = A — Protocolo:
//   Host: "G<2>" -> "<a,b,c>"
//   Arduino: "ACKA<a,b,c>" al aceptar, "OKA<m1,m2,m3>" al llegar
//
// Versión con CALIBRACIÓN por-servo (rangos PWM y grados por canal)
// ================================================================

#include <Wire.h>
#include <math.h>
#include <Adafruit_PWMServoDriver.h>

// ====== I2C Direcciones ======
#define PCA9685_ADDR     0x40
#define TCA9548A_ADDR    0x70
#define AS5600_ADDR      0x36

// ====== Canales PCA9685 (servos) ======
static const uint8_t SERVO_CH[3] = { 0, 2, 13 };   // J1, J2, J3

// ====== Canales TCA9548A (encoders) ======
static uint8_t ENCODER_CH[3] = { 0, 1, 3 };

// ====== Frecuencia del PWM ======
#define SERVO_FREQ_HZ 50  // 50 Hz

// ====== Control ======
const uint16_t CTRL_DT_MS = 125;  // 50 Hz lazo
int DIR = +1;                     // invierte global si lo necesitas

// ====== Estado ======
volatile float targetDeg[3] = {90.0f, 90.0f, 90.0f};
float startAngle[3] = {0,0,0};
float lastCorr[3]  = {0,0,0};
const float tolDeg = 0.5f;
const uint8_t REACH_HITS = 1;
uint8_t hits[3] = {0,0,0};
bool ok_sent = false;
bool busy = false;
bool control_enabled = false;   // lazo activo solo si llegaron metas

// ====== PID (IMC λ=0.40) ======
const float Kc = 0.260f;
const float Ti = 0.264f;
const float Td = 0.0645f;
const float Tf = 0.112f;

const float Ts = CTRL_DT_MS/1000.0f;
float integ[3] = {0,0,0};
float dFilt[3] = {0,0,0};
float prevMeasEff[3] = {90.0f,90.0f, 90.0f};
const float U_MIN=0.0f, U_MAX=180.0f;

// ====== PCA9685 driver ======
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(PCA9685_ADDR);
float tick_us = 0.0f;

// ====== Protocolo / Parser ======
static String in;
static uint8_t active_group = 0;

// ====== Utilidades I2C / MUX ======
static inline void tcaSelect(uint8_t ch){
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << (ch & 7));
  Wire.endTransmission();
}
static inline float clamp180(float a){ if(a<0)a=0; if(a>180)a=180; return a; }
static inline float angNorm360(float a){ while(a<0)a+=360; while(a>=360)a-=360; return a; }

// ====== Utilidades de ángulos + signo por-junta ======  // <<<
static inline float wrap180(float a){ while(a<=-180)a+=360; while(a>180)a-=360; return a; }  // <<<
static inline float angDiffDeg(float a, float b){ return wrap180(a - b); }                    // <<<
int8_t SIGN[3] = { +1, +1, -1 }; // SOLO J3 invertida (J3 = índice 2)                     // <<<

// ================================================================
// ========== CALIBRACIÓN POR-SERVO (ajústala a tus servos) =======
struct ServoCal {
  uint16_t raw_us_min, raw_us_max;   // µs donde el servo responde (crudo)
  float    raw_deg_min, raw_deg_max; // grados físicos asociados a esos µs
  float    target_deg_min, target_deg_max; // subventana física objetivo
  uint16_t hard_us_min, hard_us_max; // límites de seguridad en µs
  bool     invert;                   // invierte lógica 0..180
  float    offset_deg;               // offset lógico (+desplaza)
  float    logic_min_deg, logic_max_deg; // recorte permitido lógico
};

#define SAFE_MODE false  // si true, satura a [0..1] el mapeo RAW

// —— Calibra J1,J2,J3 aquí (valores ejemplo, AJUSTA a tu hardware) ——
ServoCal cal[3] = {
  // J1
  {650,2350,     0,   180,     0,   180,     500,2500,   false,    0.0f,   0.0f, 180.0f},
  // J2
  {650,2350,     0,   194,     0,   180,     500,2500,   false,    -10.0f,   0.0f, 180.0f},
  // J3
  {650,2350,     0,   160,     0,   180,     420,2600,   false,   -15.0f,  0.0f, 180.0f}
};

// —— OFFSETS POR TRAMO SOLO PARA J1 ————————————————————————————————
#ifndef J1_OFFSET_LO
#define J1_OFFSET_LO (-20.0f)
#endif
#ifndef J1_OFFSET_HI
#define J1_OFFSET_HI (-45.0f)
#endif

// µs -> ticks (0..4095) usando tick_us calculado en setup
uint16_t us_to_ticks(uint16_t us){
  float ticks = us / tick_us;
  if(ticks < 0) ticks = 0;
  if(ticks > 4095) ticks = 4095;
  return (uint16_t)(ticks + 0.5f);
}

static inline uint16_t clampUS_idx(uint8_t i, int us){
  if (i >= 3) return (uint16_t)us;
  if (us < cal[i].hard_us_min) us = cal[i].hard_us_min;
  if (us > cal[i].hard_us_max) us = cal[i].hard_us_max;
  return (uint16_t)us;
}

void setServoUS_idx(uint8_t i, uint16_t us){
  if (i >= 3) return;
  uint8_t ch = SERVO_CH[i];
  pca.setPWM(ch, 0, us_to_ticks(us));
}

// ———— Mapeo principal: 0..180 lógico -> µs según calibración por-servo ————
void servoWriteDeg(uint8_t i, float deg_logic_in){
  if (i >= 3) return;
  const ServoCal &c = cal[i];

  // 0) Compensación SOLO para J1: offset por tramo (sin tocar ganancia)
  float deg_logic = deg_logic_in;
  if (i == 0) {
    float off = (deg_logic_in >= 90.0f) ? J1_OFFSET_HI : J1_OFFSET_LO;
    deg_logic = deg_logic_in + off;
  } else {
    // Para J2/J3 usa el offset del struct si lo configuras
    deg_logic = deg_logic_in + c.offset_deg;
  }

  // 1) inversión lógica si aplica (calibración propia, NO la de encoder) 
  if (c.invert) deg_logic = 180.0f - deg_logic;

  // 2) recorta ventana lógica permitida
  float lmin = c.logic_min_deg, lmax = c.logic_max_deg;
  if (lmax <= lmin) { lmin = 0; lmax = 180; }
  float t_logic = (deg_logic - lmin) / (lmax - lmin);
  if (t_logic < 0) t_logic = 0; if (t_logic > 1) t_logic = 1;

  // 3) lógico -> objetivo físico (en grados físicos del servo)
  float deg_phys_target = c.target_deg_min + t_logic * (c.target_deg_max - c.target_deg_min);

  // 4) objetivo físico -> interpolación dentro de ventana RAW (grados crudos)
  float t_raw = (deg_phys_target - c.raw_deg_min) / (c.raw_deg_max - c.raw_deg_min);
  if (SAFE_MODE) { if (t_raw < 0) t_raw = 0; if (t_raw > 1) t_raw = 1; }

  int us = (int)(c.raw_us_min + t_raw * (c.raw_us_max - c.raw_us_min) + 0.5f);
  us = clampUS_idx(i, us);

  // 5) escribir al canal PCA real de ese índice
  setServoUS_idx(i, (uint16_t)us);
}

// ===================== Fin calibración por-servo =====================


// ====== Magneto bloqueante ======
void checkMagnetPresence_idx(uint8_t i){
  tcaSelect(ENCODER_CH[i]);
  uint8_t status = 0;
  while (true) {
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(0x0B);
    Wire.endTransmission();
    Wire.requestFrom(AS5600_ADDR, (uint8_t)1);
    if(Wire.available()) status = Wire.read();
    bool md = status & 0x20; // Magnet detected
    bool ml = status & 0x10; // Too weak
    bool mh = status & 0x08; // Too strong
    if (md && !ml && !mh) {
      Serial.print("# MAG"); Serial.print(i+1); Serial.println(" ok");
      break;
    }
    Serial.print("❌ Magneto no detectado en J"); Serial.println(i+1);
    delay(300);
  }
}

// ====== Lectura encoder ======
float as5600ReadDeg_idx(uint8_t i){
  tcaSelect(ENCODER_CH[i]);
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0C);
  Wire.endTransmission();
  Wire.requestFrom(AS5600_ADDR, 2);
  while(Wire.available() < 2);
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  uint16_t raw = ((uint16_t)hi << 8) | lo;
  raw &= 0x0FFF;
  float deg = raw * 0.087890625f; // 360/4096
  if(deg >= 360.0f) deg -= 360.0f;
  return deg;
}

float readCorrected_idx(uint8_t i){
  float d=as5600ReadDeg_idx(i);
  float c=d-startAngle[i]; if(c<0)c+=360; if(c>=360)c-=360;
  lastCorr[i]=c; return c;
}

// ====== NUEVO: mapea medición a 0..180 manteniendo sentido, con signo por-junta ====== // <<<
float corrToArt_idx(uint8_t i, float corr){
  // centro en TARE=90°; error respecto al centro:
  float e = angDiffDeg(corr, 90.0f);
  // aplica signo SOLO por esa junta (J3 ya viene con SIGN[2]=-1)
  e *= (float)SIGN[i];
  float a = 90.0f + e;
  if(a < 0) a = 0; if(a > 180) a = 180;
  return a;
}

// ====== TARE bloqueante ======
void verifyMagnetAndTare(){
  Serial.println("🔧 Verificando magnetos y aplicando TARE=90,90,90...");
  const float desired[3] = {90.0f, 90.0f, 90.0f};
  for(uint8_t i=0;i<3;i++){
    checkMagnetPresence_idx(i);
    float currentPos = as5600ReadDeg_idx(i);
    startAngle[i] = angNorm360(currentPos - desired[i]);
    integ[i]=0; dFilt[i]=0; prevMeasEff[i]=desired[i];
    Serial.print("# TARE J"); Serial.print(i+1);
    Serial.print(" start="); Serial.println(startAngle[i],2);
  }
  Serial.print("✅ TARE aplicado (90,90,90)  SIGN=[");
  Serial.print((int)SIGN[0]); Serial.print(",");
  Serial.print((int)SIGN[1]); Serial.print(",");
  Serial.print((int)SIGN[2]); Serial.println("]");
}

// ====== PID ======
float pid_tick(float ref, float meas, uint8_t i){
  // aplica signo efectivo = DIR global * SIGN por-junta                // <<<
  int effSign = ((DIR>0)? +1 : -1) * SIGN[i];                           // <<<
  float ref_eff  = (effSign>0)? ref  : (180.0f - ref);                  // <<<
  float meas_eff = (effSign>0)? meas : (180.0f - meas);                 // <<<
  float e = ref_eff - meas_eff;

  float dRaw = (meas_eff - prevMeasEff[i])/Ts;
  float alpha = Ts/(Tf+Ts);
  dFilt[i] += alpha*(dRaw - dFilt[i]);
  prevMeasEff[i] = meas_eff;

  float u_unsat = ref_eff + Kc*( e + (Ts/Ti)*integ[i] - Td*dFilt[i] );
  bool sat = (u_unsat>U_MAX)||(u_unsat<U_MIN);
  if(!sat || (sat && ((u_unsat>U_MAX && e<0)||(u_unsat<U_MIN && e>0))))
    integ[i] += e;
  float u = ref_eff + Kc*( e + (Ts/Ti)*integ[i] - Td*dFilt[i] );

  if(u>U_MAX)u=U_MAX; if(u<U_MIN)u=U_MIN;
  // deshacer mapeo de signo para escribir a servo                      // <<<
  float u_phys = (effSign>0)? u : (180.0f - u);                         // <<<
  return u_phys;
}

// ====== Control ======
void controlStep(){
  static unsigned long last=0, lastPrint=0;
  unsigned long now=millis();
  if(now-last<CTRL_DT_MS) return;
  last=now;

  float corr[3], meas[3], ref[3], u[3];

  for(uint8_t i=0;i<3;i++){
    corr[i]=readCorrected_idx(i);
    meas[i]=corrToArt_idx(i, corr[i]);   // <<<
    ref[i]=targetDeg[i];
  }

  // Telemetría cada ~100 ms
  if(now-lastPrint>100){
   Serial.print("# ANG<");
   Serial.print(corr[0],1); Serial.print(",");
   Serial.print(corr[1],1); Serial.print(",");
   Serial.print(corr[2],1); Serial.println(">");
   lastPrint=now;
  }

  // ===== HOLD: NO escribimos PWM si no hay metas =====
  if(!control_enabled){
    return;
  }

  // ===== PID normal =====
  for(uint8_t i=0;i<3;i++){
    u[i]=pid_tick(ref[i],meas[i],i);
    servoWriteDeg(i,u[i]);   // J1 con offset por tramo; J2/J3 normal
  }

  bool all_ok=true;
  for(uint8_t i=0;i<3;i++){
    // el mismo signo efectivo que en pid_tick:
    int effSign = ((DIR>0)? +1 : -1) * SIGN[i];                     // <<<
    float ref_eff  = (effSign>0)? ref[i]  : (180.0f - ref[i]);      // <<<
    float meas_eff = (effSign>0)? meas[i] : (180.0f - meas[i]);     // <<<
    float aerr=fabs(ref_eff-meas_eff);
    if(aerr<=tolDeg){ if(hits[i]<255) hits[i]++; }
    else { hits[i]=0; all_ok=false; }
    if(hits[i]<REACH_HITS) all_ok=false;
  }

  if(busy && !ok_sent && all_ok){
    Serial.print("OKA<");
    Serial.print(meas[0],1); Serial.print(",");
    Serial.print(meas[1],1); Serial.print(",");
    Serial.print(meas[2],1); Serial.println(">");
    ok_sent=true; busy=false;
    control_enabled=false;   // vuelve a HOLD
  }
}

// ====== Parser ======
static inline bool isLineEnd(char c){ return (c=='\n'||c=='\r'); }

bool parseAnglesInBrackets(const String& s, float& a, float& b, float& c){
  String t=s; t.trim();
  if(t.length()<5) return false;
  if(t[0]!='<' || t[t.length()-1]!='>') return false;
  String mid=t.substring(1,t.length()-1);
  for(int i=0;i<(int)mid.length();++i){
    char ch=mid[i];
    if(ch==','||ch=='\t') mid.setCharAt(i,' ');
  }
  mid.trim();
  int sp1=mid.indexOf(' '); if(sp1<0) return false;
  int sp2=mid.indexOf(' ',sp1+1); if(sp2<0) return false;

  String s1=mid.substring(0,sp1); s1.trim();
  String s2=mid.substring(sp1+1,sp2); s2.trim();
  String s3=mid.substring(sp2+1); s3.trim();

  a=s1.toFloat(); b=s2.toFloat(); c=s3.toFloat();
  return !(isnan(a)||isnan(b)||isnan(c));
}

void applyTargets(float a,float b,float c){
  targetDeg[0]=clamp180(a);
  targetDeg[1]=clamp180(b);
  targetDeg[2]=clamp180(c);

  // Reset PID consistente con nuevas metas
  for(uint8_t i=0;i<3;i++){
    integ[i]=0; dFilt[i]=0;
    // coherente con signo efectivo al entrar al lazo:                  // <<<
    int effSign = ((DIR>0)? +1 : -1) * SIGN[i];                         // <<<
    float ref_eff = (effSign>0)? targetDeg[i] : (180.0f - targetDeg[i]); // <<<
    prevMeasEff[i] = ref_eff;
    hits[i]=0;
  }
  busy=true; ok_sent=false;
  control_enabled = true;   // desde ahora hay PWM
}

void doTareAll(float desired=90.0f){
  for(uint8_t i=0;i<3;i++){
    startAngle[i]=angNorm360(as5600ReadDeg_idx(i)-desired);
    integ[i]=0; dFilt[i]=0; prevMeasEff[i]=desired;
  }
}

void parseAndApply(String s){
  s.trim(); if(!s.length()) return;

  if(s.equalsIgnoreCase("TARE")){ doTareAll(); return; }
  if(s.equalsIgnoreCase("ABORT")){ busy=false; for(uint8_t i=0;i<3;i++) hits[i]=0; ok_sent=false; control_enabled=false; Serial.println("# ACK<abort>"); return; }
  if(s.equalsIgnoreCase("DIR -1")){ DIR=-1; Serial.println("# ACK<dir=-1>"); return; }   // mantiene global si quieres
  if(s.equalsIgnoreCase("DIR 1")) { DIR=+1; Serial.println("# ACK<dir=+1>"); return; }

  if(s.equalsIgnoreCase("G<2>")){ active_group=2; Serial.println("# ACKA<G2>"); return; }
  if(s.startsWith("G<") && !s.equalsIgnoreCase("G<2>")){ Serial.println("# IGN<other_group>"); return; }

  if(active_group!=2){ Serial.println("# IGN<no_group_selected>"); return; }

  if(busy){
    if(s.startsWith("FORCE")){
      String rest=s.substring(5); rest.trim();
      float a,b,c;
      if(parseAnglesInBrackets(rest,a,b,c)) applyTargets(a,b,c);
      else Serial.println("# ERR<force_parse>");
    } else {
      Serial.print("BUSA<");
      Serial.print(targetDeg[0],1); Serial.print(",");
      Serial.print(targetDeg[1],1); Serial.print(",");
      Serial.print(targetDeg[2],1); Serial.println(">");
    }
    return;
  }

  float a,b,c;
  if(parseAnglesInBrackets(s,a,b,c)){ applyTargets(a,b,c); return; }
}

// ====== Setup / Loop ======
void setup(){
  Serial.begin(115200);
  Wire.begin(); Wire.setClock(400000L);

  pca.begin();
  pca.setPWMFreq(SERVO_FREQ_HZ);
  tick_us = 1000000.0f / (SERVO_FREQ_HZ * 4096.0f);
  
  // APAGA SALIDAS AL ARRANCAR (0% duty)
  for(uint8_t i=0;i<3;i++){
    pca.setPWM(SERVO_CH[i], 0, 0);
  }
  
  verifyMagnetAndTare(); // bloqueante, aplica TARE=90,90,90

  // Inicializa estados sin tocar PWM (HOLD real)
  for(uint8_t i=0;i<3;i++){
    float corr = readCorrected_idx(i);
    float meas = corrToArt_idx(i, corr);      // <<<
    targetDeg[i] = meas;                      // referencia interna = medición
    integ[i]=0; dFilt[i]=0;
    // coherente con signo efectivo:                                         // <<<
    int effSign = ((DIR>0)? +1 : -1) * SIGN[i];                              // <<<
    prevMeasEff[i] = (effSign>0)? meas : (180.0f - meas);                    // <<<
    hits[i]=0;
  }
  busy=false; ok_sent=false;
  control_enabled = false;      // hasta que llegue <a,b,c>

  active_group = 0;
  Serial.println("# READY (salidas OFF). Esperando G<2> y <a,b,c>");
}

void loop(){
  while(Serial.available()){
    char ch=(char)Serial.read();
    if(isLineEnd(ch)){
      if(in.length()){ parseAndApply(in); in=""; }
    } else {
      if(in.length()<96) in+=ch; else in="";
    }
  }
  controlStep();
}
