// =====================================================
//  Brazo A (único): PCA9685 + AS5600 (vía TCA9548A) + PID
//  Serial compatible con serial_node:
//   - OKA<m1,m2,m3>  => SOLO medidos (3 floats)
//   - ACKA<...>, INFO<...> => confirmaciones/diagnóstico (serial_node las ignora)
//  Flujo: recibir "<d1,d2,d3>" -> ACKA -> magnet check -> tare(90,90,0) -> PID
//  Acepta "G<2>" (compat) sin cambiar de brazo.
// =====================================================

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ====== I2C ======
#define PCA9685_ADDR  0x40
#define TCA_ADDR      0x70
#define AS5600_ADDR   0x36

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(PCA9685_ADDR);

// ====== Servos Brazo A ======
#define SERVO_A1  0
#define SERVO_A2  2
#define SERVO_A3  4

// ====== Pulsos servo (µs) y rango en grados ======
#define PWM_MIN_US   1000
#define PWM_MAX_US   2000
#define SERVO_DEG_MIN  0.0f
#define SERVO_DEG_MAX  270.0f

// ====== Mux canales encoders A1,A2,A3 ======
const uint8_t ENC_CH[3] = {0,1,2};

// ====== Utilidades TCA ======
static inline void tcaSelect(uint8_t ch){
  if (ch>7) return;
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1<<ch);
  Wire.endTransmission();
}

// ====== AS5600 ======
static inline float wrap360(float x){
  while(x<0.0f) x+=360.0f;
  while(x>=360.0f) x-=360.0f;
  return x;
}

float readAS5600Deg(uint8_t ch){
  tcaSelect(ch);
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0E); // ANGLE high
  Wire.endTransmission(false);
  Wire.requestFrom(AS5600_ADDR, (uint8_t)2);
  if (Wire.available()<2) return NAN;
  uint8_t hi=Wire.read(), lo=Wire.read();
  uint16_t raw=((uint16_t)hi<<8)|lo;
  raw&=0x0FFF;
  return (raw*360.0f)/4096.0f;
}

uint8_t magnetStatus(uint8_t ch){
  tcaSelect(ch);
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0B); // STATUS
  Wire.endTransmission(false);
  Wire.requestFrom(AS5600_ADDR,(uint8_t)1);
  if(!Wire.available()) return 0;
  uint8_t s=Wire.read();
  uint8_t MD=(s>>5)&1, ML=(s>>4)&1, MH=(s>>3)&1;
  return (MD==1 && ML==0 && MH==0) ? 1 : 0;
}

// ====== Tare ======
float enc_offset[3]={0,0,0}; // corrected = wrap360(raw - offset)

void tareToDesired(const float desired[3]){
  for(uint8_t j=0;j<3;++j){
    float raw=readAS5600Deg(ENC_CH[j]);
    if(isnan(raw)) continue;
    enc_offset[j]=wrap360(raw - desired[j]);
  }
}

float readCorrected(uint8_t j){
  float raw=readAS5600Deg(ENC_CH[j]);
  if(isnan(raw)) return NAN;
  return wrap360(raw - enc_offset[j]);
}

// ====== Servo ======
void writeServoDeg(uint8_t servo_ch, float deg){
  if(deg<SERVO_DEG_MIN) deg=SERVO_DEG_MIN;
  if(deg>SERVO_DEG_MAX) deg=SERO_DEG_MAX; // <- typo? corregimos abajo
}
