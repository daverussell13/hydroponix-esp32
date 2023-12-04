#include <WiFi.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include "secrets.h"

// Globals
WiFiManager wifiManager;
WiFiClient espClient;
PubSubClient mqttClient(espClient);

const char* DEVICE_ID = SECRET_DEVICE_ID;

const char* WIFI_SSID = "Hydroponix Device";
const char* WIFI_PASSWORD = "password";
bool isWiFiConnected = false;

const char* MQTT_HOST = "test.mosquitto.org";
const char* MQTT_USERNAME = "";
const char* MQTT_PASSWORD = "";
const int MQTT_PORT = 1883;

// Functions
void monitorWiFiConnection(void*);
void connectToMqttBroker(void*);
void mqttCallback(char*, byte*, unsigned int);

void setup() 
{
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  pinMode(LED_BUILTIN, OUTPUT);

  wifiManager.setDarkMode(true);
  wifiManager.setWiFiAutoReconnect(true);

  if(!wifiManager.autoConnect(WIFI_SSID, WIFI_PASSWORD)) 
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
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
  Serial.print("Message:");
  for(int i = 0; i < length; i++) 
  {
    Serial.print((char) payload[i]);
  }
  Serial.println();
  Serial.println("-----------------------");
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
      wifiManager.startConfigPortal(WIFI_SSID, WIFI_PASSWORD);
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