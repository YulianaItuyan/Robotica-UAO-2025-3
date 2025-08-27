#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

const char* ssid = "Pixel 8";
const char* password = "meka2205.";

WiFiServer server(10000);
WiFiClient client;
Adafruit_PWMServoDriver servos = Adafruit_PWMServoDriver(0x40);

// Calculos pulsos
unsigned int pos_0 = 172;   
unsigned int pos_180 = 565; 

// Definición canales servos
// Brazo izquierdo
const int LA_J1 = 0;  
const int LA_J2 = 1;    
const int LA_J3 = 2;  

// Brazo derecho
const int RA_J1 = 8;  
const int RA_J2 = 9;  
const int RA_J3 = 10; 

// Pata TI
const int RLL_J1 = 3; 
const int RLL_J2 = 4; 
const int RLL_J3 = 5; 

// Pata TD
const int RRL_J1 = 11;
const int RRL_J2 = 12;
const int RRL_J3 = 13;

// Pata DI
const int FLL_J1 = 6; 
const int FLL_J2 = 7; 
const int FLL_J3 = 14;

// Pata DD
const int FRL_J1 = 15;
//Otros pines
const int FRL_J2 = 16;
const int FRL_J3 = 17; 

// Manejo de conexión
unsigned long lastDataReceived = 0;
const unsigned long connectionTimeout = 60000; 
bool clientConnected = false;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin();
  servos.begin();
  servos.setPWMFreq(60);
  Serial.printf("Conectando a %s\n", ssid);
  // Conectar al wifi
  WiFi.begin(ssid, password);
  Serial.println("Conectando");
  
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  
  Serial.println();
  Serial.print("Conectado. Dirección IP: ");
  Serial.println(WiFi.localIP());
  
  // Iniciar servidor
  server.begin();
  server.setNoDelay(true); 
  Serial.printf("TCP server iniciado en puerto 10000");
}

void loop() {
  handleClientConnection();
  
  // Recibir los datos del cliente
  if (clientConnected && client.available()) {
    handleClientData();
  }
  
  checkConnectionTimeout();
  delay(10);
}

void handleClientConnection() {
  // Revisar desconexiones y conexiones nuevas
  if (!client || !client.connected()) {
    if (clientConnected) {
      Serial.println("Cliente desconectado");
      clientConnected = false;
    }
    
    // Revisar nuevas conexiones
    client = server.available();
    if (client) {
      Serial.println("Cliente conectado desde: " + client.remoteIP().toString());
      clientConnected = true;
      lastDataReceived = millis();      
    }
  }
}

void handleClientData() {
  // Leer cantidad de floats
  if (client.available() < 4) {
    return; // Esperar al menos el byte de conteo
  }
  
  uint32_t numFloats = 0;
  if (client.readBytes((char*)&numFloats, sizeof(numFloats)) != sizeof(numFloats)) {
    Serial.println("Error leyendo el contador de floats");
    return;
  }
  Serial.printf("Se esperan %d floats\n", numFloats);
  
  // Validar
  if (numFloats != 4) {
    Serial.printf("Error: Se esperaban 4 floats, llegaron %d\n", numFloats);
    // Deshacerse de datos previos
    while (client.available()) client.read();
    return;
  }
  
  // Esperar por todos los floats
  int expectedBytes = numFloats * 4;
  unsigned long timeout = millis() + 2000; 
  
  while (client.available() < expectedBytes && millis() < timeout) {
    delay(1);
  }
  
  if (client.available() < expectedBytes) {
    Serial.println("Timeout esperando, congestión en la red?");
    // Deshacerse de los datos parciales
    while (client.available()) client.read();
    return;
  }
  
  // Leer floats
  float degrees[numFloats];
  bool readSuccess = true;
  
  for (int i = 0; i < numFloats; i++) {
    // Leer 4 bytes y pasar a float
    union {
      float f;
      uint8_t bytes[4];
    } converter;
    
    if (client.readBytes(converter.bytes, 4) != 4) {
      Serial.println("Fallo al leer el float completo");
      readSuccess = false;
      break;
    }
    degrees[i] = converter.f;
  }
  
  if (!readSuccess) {
    Serial.println("Fallo en la lectura de datos");
    return;
  }
  
  // Actualizar estado de conexión
  lastDataReceived = millis();
  
  // Imprimir datos
  Serial.print("Grados recibidos: [");
  for (int i = 0; i < numFloats; i++) {
    if (i > 0) Serial.print(", ");
    Serial.print(degrees[i], 2);
  }
  Serial.println("]");
  
  // Procesar grados
  processDegreesData(degrees, numFloats);
}

void processDegreesData(float degrees[], int count) {
  if (count != 4) {
    Serial.printf("Error: Se esperaban 4 valores, se obtuvieron %d\n", count);
    return;
  }
  
  // Extraer datos, 3 angulos de articulación y 1 ID
  float joint1 = degrees[0];
  float joint2 = degrees[1];
  float joint3 = degrees[2];
  int partId = (int)degrees[3];  // Pasar a entero para seleccionar parte
  
  Serial.printf("Moviendo grupo %d: a las articulaciones en [%.2f, %.2f, %.2f]\n", 
                partId, joint1, joint2, joint3);
  
  // Switch case por cada ID
  switch (partId) {
    case 0:  // Brazo izquierdo
      moveLeftArm(joint1, joint2, joint3);
      break;
      
    case 1:  // Brazo derecho
      moveRightArm(joint1, joint2, joint3);
      break;
      
    case 2:  // Pata TI
      moveRLeftLeg(joint1, joint2, joint3);
      break;
      
    case 3:  // Pata TD
      moveRRightLeg(joint1, joint2, joint3);
      break;

    case 4:  // Pata DI
      moveFLeftLeg(joint1, joint2, joint3);
      break;

    case 5:  // Pata DD
      moveFRightLeg(joint1, joint2, joint3);
      break;
      
    default:
      Serial.printf("ID no identificado: %d\n", partId);
      break;
  }
}

void movServo(uint8_t n_servo, int angulo) {
  angulo = constrain(angulo, 0, 180);  
  int duty = map(angulo, 0, 180, pos_0, pos_180);
  servos.setPWM(n_servo, 0, duty);  
  Serial.printf("Servo canal %d movido a %d grados (duty: %d)\n", n_servo, angulo, duty);
}

// Control de servos
void moveLeftArm(float j1, float j2, float j3) {
  Serial.printf("Left arm servos: shoulder=%.2f, elbow=%.2f, wrist=%.2f\n", j1, j2, j3);
  movServo(LA_J1, (int)j1);
  delay(50);
  movServo(LA_J2, (int)j2);
  delay(50);
  movServo(LA_J3, (int)j3);
}

void moveRightArm(float j1, float j2, float j3) {
  Serial.printf("Right arm servos: shoulder=%.2f, elbow=%.2f, wrist=%.2f\n", j1, j2, j3);
  movServo(RA_J1, (int)j1);
  delay(50);
  movServo(RA_J2, (int)j2);
  delay(50);
  movServo(RA_J3, (int)j3);
}

void moveRLeftLeg(float j1, float j2, float j3) {
  Serial.printf("RL leg servos: hip=%.2f, knee=%.2f, ankle=%.2f\n", j1, j2, j3);
  movServo(RLL_J1, (int)j1);
  delay(50);
  movServo(RLL_J2, (int)j2);
  delay(50);
  movServo(RLL_J3, (int)j3);
}

void moveRRightLeg(float j1, float j2, float j3) {
  Serial.printf("RR leg servos: hip=%.2f, knee=%.2f, ankle=%.2f\n", j1, j2, j3);
  movServo(RRL_J1, (int)j1);
  delay(50);
  movServo(RRL_J2, (int)j2);
  delay(50);
  movServo(RRL_J3, (int)j3);
}

void moveFLeftLeg(float j1, float j2, float j3) {
  Serial.printf("FL leg servos: hip=%.2f, knee=%.2f, ankle=%.2f\n", j1, j2, j3);
  movServo(FLL_J1, (int)j1);
  delay(50);
  movServo(FLL_J2, (int)j2);
  delay(50);
  movServo(FLL_J3, (int)j3);
}

void moveFRightLeg(float j1, float j2, float j3) {
  Serial.printf("FR leg servos: hip=%.2f, knee=%.2f, ankle=%.2f\n", j1, j2, j3);
  movServo(FRL_J1, (int)j1);
  delay(50);
  movServo(FRL_J2, (int)j2);
  delay(50);
  movServo(FRL_J3, (int)j3);
}
void checkConnectionTimeout() {
  if (clientConnected && (millis() - lastDataReceived > connectionTimeout)) {
    Serial.println("Timeout no se recibieron datos por 60 segundos");
    client.stop();
    clientConnected = false;
  }
}
