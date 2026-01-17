#include <PS4Controller.h>

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting...");
  
  PS4.begin("c0:ff:ee:c0:ff:ee");
  
  Serial.println("Waiting for controller...");
}

void loop() {
  if (PS4.isConnected()) {
    Serial.println("CONNECTED!");
  }
  delay(100);
}