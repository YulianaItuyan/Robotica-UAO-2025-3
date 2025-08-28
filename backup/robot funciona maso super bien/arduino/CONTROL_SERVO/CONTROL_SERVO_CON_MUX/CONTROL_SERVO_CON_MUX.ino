// ====== CONFIGURACIÓN GRIPPER DC ======
#define RPWM_PIN 5       // Pin PWM para cerrar
#define LPWM_PIN 6       // Pin PWM para abrir
#define GRIPPER_SPEED 150 // Velocidad PWM (0-255)
#define finalCarrera 4 // Pin del final de carrera del gripper

// ====== SERVOS CON PCA9685 ======
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Crear objeto PCA9685
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Configuración de servos
#define SERVOMIN  150 // Valor mínimo de pulso (aproximadamente 1ms) 172
#define SERVOMAX  600 // Valor máximo de pulso (aproximadamente 2ms) 565

// Canales PCA9685 para cada brazo
// Brazo 1 (derecho)
#define SERVO1_BRAZO1  0  // Canal 0
#define SERVO2_BRAZO1  1  // Canal 1  
#define SERVO3_BRAZO1  2  // Canal 2

// Brazo 2 (izquierdo)
#define SERVO1_BRAZO2  3  // Canal 3
#define SERVO2_BRAZO2  4  // Canal 4
#define SERVO3_BRAZO2  5  // Canal 5

// Variable para saber qué brazo está activo
int brazoActivo = 1; // Por defecto brazo 1 (derecho)

unsigned long lastGripperCmd = 0;
int gripperState = 0; // 1=cerrar, -1=abrir, 0=parado

String buf;

void setup() {
  Serial.begin(115200);

  // Inicializar PCA9685
  pwm.begin();
  //pwm.setOscillatorFrequency(27000000);  // Frecuencia del oscilador interno
  pwm.setPWMFreq(60);  // Frecuencia PWM de 50Hz para servos
  
  //Wire.setClock(400000);  // Comunicación I2C rápida

  // Inicializar servos en posición central (90 grados)
  // Brazo 1 (derecho)
  servoWrite(SERVO1_BRAZO1, 90);
  servoWrite(SERVO2_BRAZO1, 90);
  servoWrite(SERVO3_BRAZO1, 90);
  
  // Brazo 2 (izquierdo)
  servoWrite(SERVO1_BRAZO2, 90);
  servoWrite(SERVO2_BRAZO2, 90);
  servoWrite(SERVO3_BRAZO2, 90);

  // Motor DC del gripper
  pinMode(RPWM_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);
  pinMode(finalCarrera, INPUT_PULLUP); // Configurar como entrada con pull-up
  analogWrite(RPWM_PIN, 0);
  analogWrite(LPWM_PIN, 0);

  Serial.println("📡 Arduino con PCA9685 listo, esperando comandos...");
  Serial.println("🤖 Brazo activo: Brazo 1 (derecho)");
  Serial.println("📍 Canales PCA9685:");
  Serial.println("   Brazo 1: canales 0, 1, 2");
  Serial.println("   Brazo 2: canales 3, 4, 5");
}

void loop() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n') {
      parseAndApply(buf);
      buf = "";
    } else {
      if (buf.length() < 64) buf += ch;
      else buf = "";
    }
  }
  
  // Control automático del gripper
  if (gripperState != 0 && millis() - lastGripperCmd > 100) {
    analogWrite(RPWM_PIN, 0);
    analogWrite(LPWM_PIN, 0);
    gripperState = 0;
  }
}

// Función para convertir ángulos (0-180) a valores PWM del PCA9685
void servoWrite(uint8_t canal, int angulo) {
  //angulo = constrain(angulo, 0, 180);
  int pulseWidth = map(angulo, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(canal, 0, pulseWidth);
}

void parseAndApply(const String& s) {
  // ====== COMANDO SELECCIÓN DE BRAZO Y GRIPPER ======
  if (s.startsWith("G<")) {
    int l = s.indexOf('<');
    int r = s.indexOf('>');
    if (l != -1 && r != -1) {
      int val = s.substring(l + 1, r).toInt();
      
      // Cambio de brazo
      if (val == 2) {
        brazoActivo = 1; // Brazo 1 (derecho)
        Serial.println("🤖 Cambiado a: Brazo 1 (derecho) - Canales 0,1,2");
        return;
      } else if (val == 3) {
        brazoActivo = 2; // Brazo 2 (izquierdo)
        Serial.println("🤖 Cambiado a: Brazo 2 (izquierdo) - Canales 3,4,5");
        return;
      }
      
      // Control del gripper (comandos 1, -1, 0)
      gripperState = val;
      lastGripperCmd = millis();

      if (val == 1) { // cerrar
        if (digitalRead(finalCarrera) == HIGH) { // Si no está en el final de carrera
          analogWrite(RPWM_PIN, GRIPPER_SPEED);
          analogWrite(LPWM_PIN, 0);
          Serial.println("🔧 Gripper cerrando...");
        } else { // Si está en el final de carrera, parar
          analogWrite(RPWM_PIN, 0);
          analogWrite(LPWM_PIN, 0);
          gripperState = 0;
          Serial.println("🔧 Gripper en final de carrera - Detenido");
        }
      } else if (val == -1) { // abrir
        analogWrite(RPWM_PIN, 0);
        analogWrite(LPWM_PIN, GRIPPER_SPEED);
        Serial.println("🔧 Gripper abriendo...");
      } else if (val == 0) { // parar
        analogWrite(RPWM_PIN, 0);
        analogWrite(LPWM_PIN, 0);
        Serial.println("🔧 Gripper detenido");
      }
    }
    return;
  }

  // ====== COMANDO BRAZO ======
  int l = s.indexOf('<');
  int r = s.indexOf('>');
  if (l == -1 || r == -1 || r <= l) return;

  String payload = s.substring(l + 1, r);
  int c1 = payload.indexOf(',');
  int c2 = payload.indexOf(',', c1 + 1);
  if (c1 == -1 || c2 == -1) return;

  int v1 = payload.substring(0, c1).toInt();
  int v2 = payload.substring(c1 + 1, c2).toInt();
  int v3 = payload.substring(c2 + 1).toInt();

  v1 = constrain(v1, 0, 180);
  v2 = constrain(v2, 0, 180);
  v3 = constrain(v3, 0, 180);

  // Enviar comandos al brazo activo
  if (brazoActivo == 1) {
    // Brazo 1 (derecho) - Canales 0, 1, 2
    servoWrite(SERVO1_BRAZO1, v1);
    servoWrite(SERVO2_BRAZO1, v2);
    servoWrite(SERVO3_BRAZO1, v3);
    Serial.print("🤖 Brazo 1 (canales 0,1,2) - Recibido: ");
  } else {
    // Brazo 2 (izquierdo) - Canales 3, 4, 5

    servoWrite(SERVO1_BRAZO2, v1); // Invertir movimiento para brazo izquierdo
    servoWrite(SERVO2_BRAZO2, 180 - v2); // Invertir movimiento para brazo izquierdo
    servoWrite(SERVO3_BRAZO2, 180 - v3); // Invertir movimiento para brazo izquierdo
    Serial.print("🤖 Brazo 2 (canales 3,4,5) - Recibido: ");
  }
  
  Serial.println(payload);
}