// ====== CONFIGURACIÓN GRIPPER DC ======
#define RPWM_PIN 5       // Pin PWM para cerrar
#define LPWM_PIN 6       // Pin PWM para abrir
#define GRIPPER_SPEED 150 // Velocidad PWM (0-255)
#define finalCarrera 4 // Pin del final de carrera del gripper

// ====== SERVOS DEL BRAZO ======
#include <Servo.h>


unsigned long lastGripperCmd = 0;
int gripperState = 0; // 1=cerrar, -1=abrir, 0=parado

Servo s1, s2, s3;

String buf;

void setup() {
  Serial.begin(115200);

  // Servos del brazo
  s1.attach(9);
  s2.attach(10);
  s3.attach(11);
  s1.write(90); s2.write(90); s3.write(90);

  // Motor DC del gripper
  pinMode(RPWM_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);
  analogWrite(RPWM_PIN, 0);
  analogWrite(LPWM_PIN, 0);

  Serial.println("📡 Arduino listo, esperando comandos...");
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
  if (gripperState != 0 && millis() - lastGripperCmd > 100) {
    analogWrite(RPWM_PIN, 0);
    analogWrite(LPWM_PIN, 0);
    gripperState = 0;
  }
}

void parseAndApply(const String& s) {
  // ====== COMANDO GRIPPER ======
  if (s.startsWith("G<")) {
    int l = s.indexOf('<');
    int r = s.indexOf('>');
    if (l != -1 && r != -1) {
      int val = s.substring(l + 1, r).toInt();
      gripperState = val;
      lastGripperCmd = millis();

      if (val == 1) { // cerrar
        if (digitalRead(finalCarrera) == HIGH) { // Si no está en el final de carrera
          analogWrite(RPWM_PIN, GRIPPER_SPEED);
          analogWrite(LPWM_PIN, 0);
        } else { // Si está en el final de carrera, parar
          analogWrite(RPWM_PIN, 0);
          analogWrite(LPWM_PIN, 0);
          gripperState = 0;  // no estoy seguro de si eso va
        }
        // analogWrite(RPWM_PIN, GRIPPER_SPEED);
        // analogWrite(LPWM_PIN, 0);
      } else if (val == -1) { // abrir
        analogWrite(RPWM_PIN, 0);
        analogWrite(LPWM_PIN, GRIPPER_SPEED);
      } else { // parar
        analogWrite(RPWM_PIN, 0);
        analogWrite(LPWM_PIN, 0);
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

  s1.write(v1);
  s2.write(v2);
  s3.write(v3);

  Serial.print("Recibido: ");
  Serial.println(payload);
}
