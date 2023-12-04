#include <WiFi.h>
#include <WiFiManager.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include "secrets.h"

// Globals
WiFiManager wifiManager;
WiFiClient espClient;
PubSubClient mqttClient(espClient);

const char* DEVICE_ID = SECRET_DEVICE_ID;

const char* AP_SSID = "Hydroponix Device";
const char* AP_PASSWORD = SECRET_AP_PASSWORD;
bool isWiFiConnected = false;

const char* MQTT_HOST = "test.mosquitto.org";
const int MQTT_PORT = 1883;
const char* MQTT_USERNAME = SECRET_MQTT_USERNAME;
const char* MQTT_PASSWORD = SECRET_MQTT_PASSWORD;

// MQTT Topics
const char* SENSOR_TOPIC = "hydroponix/sensor";
const char* PUMP_TOPIC = "hydroponix/pump";

// Functions
void monitorWiFiConnection(void*);
void connectToMqttBroker(void*);
void mqttCallback(char*, byte*, unsigned int);
void mqttPumpHandler(String);

void setup() 
{
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  pinMode(LED_BUILTIN, OUTPUT);

  wifiManager.setDarkMode(true);
  wifiManager.setWiFiAutoReconnect(true);

  if(!wifiManager.autoConnect(AP_SSID, AP_PASSWORD)) 
  {
    Serial.println("Failed to connect");
  }
  else
  {
    Serial.println("Connected to " + wifiManager.getWiFiSSID());
  }

  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);

  xTaskCreatePinnedToCore(monitorWiFiConnection, "Monitor WiFi Connection", 1024, NULL, 2, NULL, CONFIG_ARDUINO_RUNNING_CORE);
  xTaskCreate(connectToMqttBroker, "Connect to MQTT Broker", 1024 * 2, NULL, 1, NULL);
}

void loop() 
{
  StaticJsonDocument<200> sensorJDoc;
  sensorJDoc["ph"] = 7;
  sensorJDoc["temp"] = 30;
  sensorJDoc["tds"] = 130;

  String sensorJSON;
  serializeJson(sensorJDoc, sensorJSON);

  mqttClient.publish(SENSOR_TOPIC, sensorJSON.c_str());
  delay(5000);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) 
{
  String msgBuff;
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);

  for (int i = 0; i < length; i++) {
    msgBuff += (char) payload[i];
  }

  if (!strcmp(topic, PUMP_TOPIC)) mqttPumpHandler(msgBuff);
}

void monitorWiFiConnection(void* params)
{
  while(true)
  {
    if(WiFi.isConnected()) 
    {
      digitalWrite(LED_BUILTIN, HIGH);
      isWiFiConnected = true;
      wifiManager.stopConfigPortal();
    }
    else
    {
      digitalWrite(LED_BUILTIN, LOW);
      isWiFiConnected = false;
      wifiManager.startConfigPortal(AP_SSID, AP_PASSWORD);
    }
    vTaskDelay(250 / portTICK_PERIOD_MS);
  }
}

void connectToMqttBroker(void* params)
{
  while(true)
  {
    int delay_ms = 500;
    if(isWiFiConnected && mqttClient.connected())
    {
      mqttClient.loop();        
    }
    else if(isWiFiConnected)
    {
      Serial.println("Attempting to connect MQTT broker");
      if(mqttClient.connect(DEVICE_ID, MQTT_USERNAME, MQTT_PASSWORD))
      {
        Serial.println("Connected to MQTT broker");
        mqttClient.subscribe(PUMP_TOPIC);
      }
      else
      {
        Serial.println("Failed connecting to MQTT broker");
        Serial.println("Retry in 5 seconds");
        delay_ms = 5000;
      }
    }
    vTaskDelay(delay_ms / portTICK_PERIOD_MS);
  }
}

void mqttPumpHandler(String msg)
{
  // Triggering pump
  Serial.println(msg);
}