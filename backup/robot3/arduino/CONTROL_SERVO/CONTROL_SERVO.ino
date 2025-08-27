#include <Servo.h>

Servo s1, s2, s3;
int gripper_pos = 90; // Posición inicial del gripper (ajústala si es necesario)
Servo gripper_servo;  // Objeto para el servo del gripper

String buf;

void setup() {
  Serial.begin(115200);
  s1.attach(9);
  s2.attach(10);
  s3.attach(11);
  gripper_servo.attach(12); // Pin para el servo del gripper
  s1.write(90); s2.write(90); s3.write(90);
  gripper_servo.write(gripper_pos);

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
}

void parseAndApply(const String& s) {
  // Maneja el comando del gripper
  if (s.startsWith("G<")) {
    int l = s.indexOf('<');
    int r = s.indexOf('>');
    if (l != -1 && r != -1) {
      // Ignoramos el valor y solo damos un pequeño pulso de movimiento
      gripper_pos += 5; // Mueve el gripper 5 grados para cerrar (ajusta este valor)
      gripper_pos = constrain(gripper_pos, 0, 180);
      gripper_servo.write(gripper_pos);
      Serial.print("Comando Gripper: moviendo a ");
      Serial.print(gripper_pos);
      Serial.println("°");
    }
    return; // Importante para no seguir con el resto de la función
  }
  
  // Código original para los servos del brazo
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

  // Mover servos
  s1.write(v1);
  s2.write(v2);
  s3.write(v3);

  Serial.print("Recibido: ");
  Serial.println(payload);
  Serial.print("Aplicado -> S1: ");
  Serial.print(v1);
  Serial.print("°, S2: ");
  Serial.print(v2);
  Serial.print("°, S3: ");
  Serial.print(v3);
  Serial.println("°");
}