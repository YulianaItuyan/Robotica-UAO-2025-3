// ——— 3x Servo (PCA9685) + 3x AS5600 (via TCA9548A) + PID(IMC) ———
// Objetivo: Mp ≤ 5%, tp ≈ 0.5 s (lambda=0.40); robusto con retardo L

#include <Wire.h>
#include <Servo.h> // (no usado para PCA, pero lo dejo si compilas junto a otros tests)
#include <math.h>
#include <Adafruit_PWMServoDriver.h>

// ===== Direcciones I2C =====
#define PCA9685_ADDR     0x40
#define TCA9548A_ADDR    0x70
#define AS5600_ADDR      0x36

// ===== Canales PCA9685 (servos) =====
static const uint8_t SERVO_CH[3] = { 0, 2, 4 };   // <— AJUSTA: canales PCA para J1, J2, J3

// ===== Canales TCA9548A (encoders) =====
static uint8_t ENCODER_CH[3] = { 0, 1, 2 };       // <— AJUSTA: canales mux para AS5600 de J1, J2, J3

// ===== Pulsos servo (us) y frecuencia =====
#define PWM_MIN_US       1000
#define PWM_MAX_US       2000
#define SERVO_FREQ_HZ    50      // 50 Hz (20 ms)

// ===== Control =====
const uint16_t CTRL_DT_MS = 20;  // 50 Hz lazo
int DIR = +1;                    // mismo sentido para las 3 (si todo va al revés: DIR=-1)

// ===== Estado por articulación =====
volatile float targetDeg[3] = {90.0f, 90.0f, 90.0f}; // 0..180
float startAngle[3] = {0,0,0};    // cero (tare) por conjunto
float lastCorr[3]  = {0,0,0};
const float tolDeg = 0.5f;
const uint8_t REACH_HITS = 3;
uint8_t hits[3] = {0,0,0};
bool ok_sent = false;
bool busy = false;

String buf;

// ===== PID (IMC λ=0.40) — idéntico por articulación =====
const float Kc = 0.335f;
// const float Kc = 0.280f;      // opción más lenta
const float Ti = 0.264f;
const float Td = 0.0645f;        // (tu valor más reciente)
const float Tf = 0.112f;

const float Ts = CTRL_DT_MS/1000.0f;
float integ[3] = {0,0,0};
float dFilt[3] = {0,0,0};
float prevMeasEff[3] = {90.0f,90.0f,90.0f};

const float U_MIN=0.0f, U_MAX=180.0f;

// ===== PCA9685 driver =====
Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(PCA9685_ADDR);
float tick_us = 0.0f;  // microsegundos por tick (calculado tras setPWMFreq)

// ===== Utilidades =====
static inline void tcaSelect(uint8_t ch){
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << (ch & 7));
  Wire.endTransmission();
}
static inline float clamp180(float a){ if(a<0)a=0; if(a>180)a=180; return a; }

// microsegundos -> ticks (0..4095) a SERVO_FREQ_HZ
uint16_t us_to_ticks(uint16_t us){
  float ticks = us / tick_us;           // tick_us = 1e6 / (SERVO_FREQ_HZ*4096)
  if(ticks < 0) ticks = 0;
  if(ticks > 4095) ticks = 4095;
  return (uint16_t)(ticks + 0.5f);
}

// ángulo (0..180) -> microsegundos lineal
uint16_t deg_to_us(float deg){
  deg = clamp180(deg);
  return (uint16_t)(PWM_MIN_US + (PWM_MAX_US - PWM_MIN_US) * (deg/180.0f));
}

// escribir un servo (por índice 0..2)
void servoWriteDeg(uint8_t i, float ang){
  uint16_t us = deg_to_us(ang);
  uint16_t off = us_to_ticks(us);
  pca.setPWM(SERVO_CH[i], 0, off);
}

float as5600ReadDeg_idx(uint8_t i){
  tcaSelect(ENCODER_CH[i]); // asegurar canal activo para ese encoder

  // LOW
  Wire.beginTransmission(AS5600_ADDR); Wire.write(0x0D); Wire.endTransmission();
  Wire.requestFrom(AS5600_ADDR,1); while(!Wire.available()); uint8_t lo=Wire.read();

  // HIGH
  Wire.beginTransmission(AS5600_ADDR); Wire.write(0x0C); Wire.endTransmission();
  Wire.requestFrom(AS5600_ADDR,1); while(!Wire.available()); uint8_t hi=Wire.read();

  uint16_t raw=((uint16_t)hi<<8)|lo; 
  float deg=raw*0.087890625f; // 360/4096
  if(deg>=360)deg-=360;
  return deg;
}

bool as5600CheckMagnet_idx(uint8_t i){
  tcaSelect(ENCODER_CH[i]);
  Wire.beginTransmission(AS5600_ADDR); Wire.write(0x0B); Wire.endTransmission();
  Wire.requestFrom(AS5600_ADDR,1); while(!Wire.available()); uint8_t st=Wire.read();
  bool md = st & 0x20; // Magnet Detected
  bool ml = st & 0x10; // Too Weak
  bool mh = st & 0x08; // Too Strong

  if(md && !ml && !mh){
    Serial.print("MAG"); Serial.print(i+1); Serial.println("<ok>");
    return true;
  }
  Serial.print("MAG"); Serial.print(i+1); Serial.print("<");
  if(!md) Serial.print("no_magnet");
  if(ml)  Serial.print(md?":weak":"weak");
  if(mh)  Serial.print((!md&&!ml)?"strong":":strong");
  Serial.println(">");
  return false;
}

float readCorrected_idx(uint8_t i){
  float d=as5600ReadDeg_idx(i);
  float c=d-startAngle[i]; if(c<0)c+=360; if(c>=360)c-=360; lastCorr[i]=c; return c;
}

// “plegado” 0..180
float corrToArt(float corr){
  float a=corr; if(a<0)a+=360; if(a>=360)a-=360; if(a>180)a=360-a; return a;
}

void doTare_idx(uint8_t i, float desired=90.0f){
  float pos=as5600ReadDeg_idx(i);
  startAngle[i]=pos-desired; if(startAngle[i]<0)startAngle[i]+=360; if(startAngle[i]>=360)startAngle[i]-=360;
  integ[i]=0; dFilt[i]=0; prevMeasEff[i]=desired; hits[i]=0; ok_sent=false;
  Serial.print("TARE"); Serial.print(i+1); Serial.print("<start="); Serial.print(startAngle[i],2); Serial.println(">");
}

// ===== PID tick — idéntico a tu lógica =====
float pid_tick(float ref, float meas, uint8_t i){
  float ref_eff  = (DIR>0)? ref  : (180.0f - ref);
  float meas_eff = (DIR>0)? meas : (180.0f - meas);

  float e = ref_eff - meas_eff;

  // derivada filtrada (1er orden) sobre la medición
  float dRaw = (meas_eff - prevMeasEff[i])/Ts;
  float alpha = Ts/(Tf+Ts);
  dFilt[i] += alpha*(dRaw - dFilt[i]);
  prevMeasEff[i] = meas_eff;

  // integrador con anti-windup simple
  float u_unsat = ref_eff + Kc*( e + (Ts/Ti)*integ[i] - Td*dFilt[i] );
  bool sat = (u_unsat>U_MAX)||(u_unsat<U_MIN);
  if(!sat || (sat && ((u_unsat>U_MAX && e<0)||(u_unsat<U_MIN && e>0)))){
    integ[i] += e;
  }
  float u = ref_eff + Kc*( e + (Ts/Ti)*integ[i] - Td*dFilt[i] );

  if(u>U_MAX)u=U_MAX; if(u<U_MIN)u=U_MIN;
  float u_phys = (DIR>0)? u : (180.0f - u);
  return u_phys;
}

// ===== Control =====
void controlStep(){
  static unsigned long last=0, lastPrint=0; unsigned long now=millis();
  if(now-last<CTRL_DT_MS) return; last=now;

  float corr[3], meas[3], ref[3], u[3];

  for(uint8_t i=0;i<3;i++){
    corr[i]=readCorrected_idx(i);
    meas[i]=corrToArt(corr[i]);
    ref[i]=targetDeg[i];
    u[i]=pid_tick(ref[i],meas[i],i);
    servoWriteDeg(i,u[i]);
  }

  if(now-lastPrint>100){
    Serial.print("ANG<");
    Serial.print(corr[0],1); Serial.print(",");
    Serial.print(corr[1],1); Serial.print(",");
    Serial.print(corr[2],1); Serial.println(">");
    lastPrint=now;
  }

  // verificación de llegada (todas)
  bool all_ok = true;
  for(uint8_t i=0;i<3;i++){
    float ref_eff  = (DIR>0)? ref[i]  : (180.0f - ref[i]);
    float meas_eff = (DIR>0)? meas[i] : (180.0f - meas[i]);
    float aerr=fabs(ref_eff-meas_eff);
    if(aerr<=tolDeg){ if(hits[i]<255) hits[i]++; }
    else { hits[i]=0; all_ok=false; }
    if(hits[i]<REACH_HITS) all_ok=false;
  }

  if(!ok_sent && all_ok){
    Serial.print("OK<");
    Serial.print(meas[0],1); Serial.print(",");
    Serial.print(meas[1],1); Serial.print(",");
    Serial.print(meas[2],1); Serial.println(">");
    ok_sent=true; busy=false;
  }
}

// ===== Parser =====
void applyTargets(float a, float b, float c){
  targetDeg[0]=clamp180(a);
  targetDeg[1]=clamp180(b);
  targetDeg[2]=clamp180(c);
  busy=true; ok_sent=false;
  for(uint8_t i=0;i<3;i++) hits[i]=0;

  Serial.print("ACK<");
  Serial.print(targetDeg[0],1); Serial.print(",");
  Serial.print(targetDeg[1],1); Serial.print(",");
  Serial.print(targetDeg[2],1); Serial.println(">");
}

static inline bool isLineEnd(char c){ return (c=='\n'||c=='\r'); }

// acepta "a b c" ó "a,b,c"
bool parse3floats(const String& s, float& a, float& b, float& c){
  String t=s; t.trim(); if(!t.length()) return false;
  char sep = (t.indexOf(',')>=0)? ',' : ' ';
  // normalizar separador a espacio
  String n;
  for(size_t i=0;i<t.length();++i){
    char ch=t[i];
    if(ch==',' || ch=='\t') ch=' ';
    n+=ch;
  }
  int sp1 = n.indexOf(' ');
  if(sp1<0) return false;
  int sp2 = n.indexOf(' ', sp1+1);
  if(sp2<0) return false;

  String s1 = n.substring(0,sp1); s1.trim();
  String s2 = n.substring(sp1+1,sp2); s2.trim();
  String s3 = n.substring(sp2+1); s3.trim();

  a = s1.toFloat(); b = s2.toFloat(); c = s3.toFloat();
  return !(isnan(a)||isnan(b)||isnan(c));
}

void doTareAll(float desired=90.0f){
  for(uint8_t i=0;i<3;i++) doTare_idx(i, desired);
}

void parseAndApply(String s){
  s.trim(); if(!s.length())return;

  if(s.equalsIgnoreCase("TARE")){ doTareAll(); return; }
  if(s.equalsIgnoreCase("ABORT")){ busy=false; for(uint8_t i=0;i<3;i++) hits[i]=0; ok_sent=false; Serial.println("ACK<abort>"); return; }
  if(s.equalsIgnoreCase("DIR -1")){ DIR=-1; Serial.println("ACK<dir=-1>"); return; }
  if(s.equalsIgnoreCase("DIR 1")) { DIR=+1; Serial.println("ACK<dir=+1>"); return; }

  float a,b,c;

  if(busy){
    if(s.startsWith("FORCE")){
      String rest = s.substring(5); rest.trim();
      if(parse3floats(rest,a,b,c)) applyTargets(a,b,c);
    } else {
      Serial.print("BUSY<");
      Serial.print(targetDeg[0],1); Serial.print(",");
      Serial.print(targetDeg[1],1); Serial.print(",");
      Serial.print(targetDeg[2],1); Serial.println(">");
    }
    return;
  }

  if(parse3floats(s,a,b,c)) applyTargets(a,b,c);
  // si no son 3 floats, ignorar (o podrías imprimir ayuda)
}

// ===== Setup / Loop =====
void setup(){
  Serial.begin(115200);
  Wire.begin(); Wire.setClock(400000L);

  // PCA9685 init
  pca.begin();
  pca.setPWMFreq(SERVO_FREQ_HZ);
  tick_us = 1000000.0f / (SERVO_FREQ_HZ * 4096.0f);

  // Check magnet de los 3 (hasta 1 s cada uno, no bloqueante infinito)
  for(uint8_t i=0;i<3;i++){
    unsigned long t0 = millis(); bool ok=false;
    while(!(ok = as5600CheckMagnet_idx(i)) && (millis()-t0<1000)) delay(100);
  }

  // Llevar los 3 servos a 90°
  for(uint8_t i=0;i<3;i++) servoWriteDeg(i, 90.0f);
  delay(400);

  

  // TARE de los 3 a 90° y objetivo inicial (90,90,90)
  doTareAll(90.0f);
  applyTargets(90.0f, 90.0f, 90.0f);

  Serial.println("CMDS<'a b c' | 'a,b,c' | TARE | ABORT | FORCE a b c | DIR 1 | DIR -1>");
}

void loop(){
  static String in;
  while(Serial.available()){
    char ch=(char)Serial.read();
    if(isLineEnd(ch)){ if(in.length()){ parseAndApply(in); in=""; } }
    else { if(in.length()<96) in+=ch; else in=""; }
  }
  controlStep();
}