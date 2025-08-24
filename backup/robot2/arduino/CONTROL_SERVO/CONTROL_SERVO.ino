#include <Servo.h>

int motorPin = 6;
Servo s1, s2, s3;


String buf;
int a = 90, b = 90, c = 90;

void setup() {
  Serial.begin(115200);
  pinMode(motorPin, OUTPUT);
  digitalWrite(motorPin, LOW); // Activar alimentación de servos
  s1.attach(9);
  s2.attach(10);
  s3.attach(11);
  s1.write(a); s2.write(b); s3.write(c);

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
  int l = s.indexOf('<');
  int r = s.indexOf('>');
  if (l == -1 || r == -1 || r <= l) return;

  String payload = s.substring(l + 1, r);

   // Si contiene "G"
  if (payload.indexOf("G1") != -1) {
    digitalWrite(motorPin, HIGH);  // enciende motor
    delay(200);                    // 200 ms encendido (ajusta)
    digitalWrite(motorPin, LOW);   // apaga motor
    Serial.println("⚡ Pulsador gripper activado");
    return;
  }

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

  // 💬 Mostrar lo recibido y aplicado
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
