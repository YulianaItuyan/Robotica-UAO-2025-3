#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>

// Configuración WiFi
const char* ssid = "iPhone de Sergio Andres";
const char* password = "mambobatiri";
const char* mqtt_server = "test.mosquitto.org";  

// Configuración MQTT
//const char* mqtt_server = "192.168.1.100";  // IP de tu PC con ROS2
const int mqtt_port = 1883;
const char* mqtt_topic = "robot/servo_cmd";
const char* mqtt_client_id = "ESP32_Robot";

// Pines de servos (ajusta según tu conexión)
const int SERVO1_PIN = 18;
const int SERVO2_PIN = 19; 
const int SERVO3_PIN = 23;

// Objetos
WiFiClient espClient;
PubSubClient mqtt_client(espClient);
Servo servo1, servo2, servo3;

// Variables
int current_pos[3] = {90, 90, 90};  // Posiciones actuales
unsigned long last_msg = 0;
bool wifi_connected = false;
bool mqtt_connected = false;

void setup() {
  Serial.begin(115200);
  Serial.println("🤖 ESP32 Robot Controller iniciando...");
  
  // Configurar servos
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);
  
  // Posición inicial
  servo1.write(90);
  servo2.write(90);
  servo3.write(90);
  
  Serial.println("✅ Servos inicializados en 90°");
  
  // Conectar WiFi
  setup_wifi();
  
  // Configurar MQTT
  mqtt_client.setServer(mqtt_server, mqtt_port);
  mqtt_client.setCallback(mqtt_callback);
  
  Serial.println("📡 Sistema listo!");
}

void setup_wifi() {
  delay(10);
  Serial.print("🔗 Conectando a WiFi: ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    wifi_connected = true;
    Serial.println();
    Serial.println("✅ WiFi conectado!");
    Serial.print("📍 IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    wifi_connected = false;
    Serial.println();
    Serial.println("❌ Error conectando WiFi");
  }
}

void reconnect_mqtt() {
  while (!mqtt_client.connected() && wifi_connected) {
    Serial.print("🔄 Conectando a MQTT...");
    
    if (mqtt_client.connect(mqtt_client_id)) {
      mqtt_connected = true;
      Serial.println(" ✅ Conectado!");
      mqtt_client.subscribe(mqtt_topic);
      Serial.print("📥 Suscrito a: ");
      Serial.println(mqtt_topic);
    } else {
      mqtt_connected = false;
      Serial.print(" ❌ Error, rc=");
      Serial.print(mqtt_client.state());
      Serial.println(" reintentando en 5 segundos");
      delay(5000);
    }
  }
}

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  // Convertir payload a string
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.print("📨 Mensaje recibido [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(message);
  
  // Parsear JSON
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, message);
  
  if (error) {
    Serial.print("❌ Error parseando JSON: ");
    Serial.println(error.c_str());
    return;
  }
  
  // Extraer valores
  int servo1_pos = doc["servo1"] | 90;  // valor por defecto 90
  int servo2_pos = doc["servo2"] | 90;
  int servo3_pos = doc["servo3"] | 90;
  
  // Validar rangos
  servo1_pos = constrain(servo1_pos, 0, 180);
  servo2_pos = constrain(servo2_pos, 0, 180);
  servo3_pos = constrain(servo3_pos, 0, 180);
  
  // Aplicar movimientos
  servo1.write(servo1_pos);
  servo2.write(servo2_pos);
  servo3.write(servo3_pos);
  
  // Actualizar posiciones actuales
  current_pos[0] = servo1_pos;
  current_pos[1] = servo2_pos;
  current_pos[2] = servo3_pos;
  
  // Mostrar resultado
  Serial.print("🎯 Servos actualizados -> S1: ");
  Serial.print(servo1_pos);
  Serial.print("°, S2: ");
  Serial.print(servo2_pos);
  Serial.print("°, S3: ");
  Serial.print(servo3_pos);
  Serial.println("°");
}

void loop() {
  // Verificar WiFi
  if (WiFi.status() != WL_CONNECTED && wifi_connected) {
    wifi_connected = false;
    Serial.println("⚠️ WiFi desconectado, reintentando...");
    setup_wifi();
  }
  
  // Verificar MQTT
  if (!mqtt_client.connected() && wifi_connected) {
    mqtt_connected = false;
    reconnect_mqtt();
  }
  
  // Procesar mensajes MQTT
  if (mqtt_connected) {
    mqtt_client.loop();
  }
  
  // Status cada 30 segundos
  unsigned long now = millis();
  if (now - last_msg > 30000) {
    last_msg = now;
    Serial.print("📊 Status - WiFi: ");
    Serial.print(wifi_connected ? "✅" : "❌");
    Serial.print(" | MQTT: ");
    Serial.print(mqtt_connected ? "✅" : "❌");
    Serial.print(" | Servos: [");
    Serial.print(current_pos[0]);
    Serial.print(", ");
    Serial.print(current_pos[1]);
    Serial.print(", ");
    Serial.print(current_pos[2]);
    Serial.println("]");
  }
  
  delay(10);  // Small delay to prevent watchdog issues
}