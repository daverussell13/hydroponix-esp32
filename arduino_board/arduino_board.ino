#include <SoftwareSerial.h>
#include <ArduinoJson.h>

SoftwareSerial ArduinoSerialCom(2, 3);

// Pump relay
const int tdsBufferRelay = 7;
const unsigned long tdsRelayDuration = 2000; // 2 seconds

void setup() {
  Serial.begin(9600);
  ArduinoSerialCom.begin(9600);
  pinMode(tdsBufferRelay, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(tdsBufferRelay, HIGH);
}

void loop() {
  if (ArduinoSerialCom.available() > 0) 
  {
    String cmd = ArduinoSerialCom.readString();
    DynamicJsonDocument doc(200);
    DeserializationError error = deserializeJson(doc, cmd);
    if (error == DeserializationError::Ok) 
    {
      if (doc.containsKey("trigger") && doc["trigger"].is<String>()) 
      {
        if (doc["trigger"] == "tds") 
        {
          digitalWrite(tdsBufferRelay, LOW);
          digitalWrite(LED_BUILTIN, HIGH);
          delay(2000);
          digitalWrite(tdsBufferRelay, HIGH);
          digitalWrite(LED_BUILTIN, LOW);
        }
      }
    }
  }
}