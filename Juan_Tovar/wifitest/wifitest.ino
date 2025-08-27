#include <WiFi.h>

const char* ssid = "Pixel 8";
const char* password = "meka2205.";

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  
  Serial.println("Connected!");
  Serial.println(WiFi.localIP());
}

void loop() {
  delay(1000);
}
