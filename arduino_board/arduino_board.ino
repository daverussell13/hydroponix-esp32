#include <SoftwareSerial.h>
#include <ArduinoJson.h>

SoftwareSerial ArduinoSerialCom(2, 3);

// Pump relay
const int tdsBufferRelay = 6;
bool isPumpActive = false;

const unsigned long tdsRelayDuration = 2000; // 2 seconds
const unsigned long ledDuration = 2000; // 2 seconds

unsigned long lastTdsTime = 0;
unsigned long lastLedTime = 0;

void setup()
{
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
          delay(tdsRelayDuration);
          digitalWrite(tdsBufferRelay, HIGH);
        }
        else if (doc["trigger"] == "led")
        {
          digitalWrite(LED_BUILTIN, HIGH);
          delay(ledDuration);
          digitalWrite(LED_BUILTIN, LOW);
        }
      }
    }
  }
}