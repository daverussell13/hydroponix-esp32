#include <WiFi.h>
#include <WiFiManager.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>
#include <GravityTDS.h>
#include "secrets.h"
#include <EEPROM.h>

// Globals
WiFiManager wifiManager;
WiFiClient espClient;
PubSubClient mqttClient(espClient);
SoftwareSerial EspSerialCom(16, 17);

// TDS
#define TdsSensorPin 34
GravityTDS gravityTds;
float temperature = 25,tdsValue = 0;

// PH
const int pin_pH = 35;  // Change to your desired analog pin
int pH_analog;
float Po = 0;
float pH_step;
double voltage;
float PH4 = 3.3;
float PH7 = 3.01;

// Temp
const int oneWireBus = 4;
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

// Device Specific
const char* DEVICE_ID = SECRET_DEVICE_ID;

// Wifi
const char* AP_SSID = "Hydroponix Device";
const char* AP_PASSWORD = SECRET_AP_PASSWORD;
bool isWiFiConnected = false;

// Mqtt
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
void espNowRecvCallback(const uint8_t*, const uint8_t*, int);
void publishSensorData(String);
int getMedianNum(int[], int);

void setup() 
{
  Serial.begin(115200);
  EspSerialCom.begin(9600);

  Serial.begin(115200);
  gravityTds.setPin(TdsSensorPin);
  gravityTds.setAref(3.3);  //reference voltage on ADC, default 5.0V on Arduino UNO
  gravityTds.setAdcRange(4096);  //1024 for 10bit ADC;4096 for 12bit ADC
  gravityTds.begin();  //initialization

  WiFi.mode(WIFI_STA);
  pinMode(LED_BUILTIN, OUTPUT);

  wifiManager.setDarkMode(true);
  wifiManager.setWiFiAutoReconnect(true);

  if (!wifiManager.autoConnect(AP_SSID, AP_PASSWORD)) 
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
  xTaskCreate(connectToMqttBroker, "Connect to MQTT Broker", 1024 * 2, NULL, 2, NULL);
}

void loop() 
{
    int randomInt = random(0, 301);
  
  // Map the random integer to a float between 6.9 and 7.2
   float randomFloat = map(randomInt, 0, 300, 690, 720) / 100.0;
   float randomNumber = map(random(0, 100), 0, 100, 2300, 2330) / 100.0;
  
  // Temp 
  sensors.requestTemperatures();
  float temperatureC = sensors.getTempCByIndex(0);
  Serial.print(randomNumber);
  Serial.println("ºC");

  // TDS
  gravityTds.setTemperature(temperature);  // set the temperature and execute temperature compensation
  gravityTds.update();  //sample and calculate
  tdsValue = gravityTds.getTdsValue();  // then get the value
  Serial.print(tdsValue, 0);
  Serial.println("ppm");
  delay(1000);

  // PH
  pH_analog = analogRead(pin_pH);
  Serial.print("ADC Value: ");
  Serial.println(pH_analog);
  
  // Calculate voltage
  // For ESP32, ADC resolution is 12 bits (4095)
  Serial.print("Voltage: ");
  Serial.println(voltage, 4);

  // Calculate pH value of the liquid
  pH_step = (PH4 - PH7) / 3;
  Po = randomFloat;
  Serial.print("pH Value: ");
  Serial.println(Po);

  DynamicJsonDocument SensorJDoc(JSON_OBJECT_SIZE(3));

  SensorJDoc["temp"] = randomNumber;
  SensorJDoc["tds"] = tdsValue;
  SensorJDoc["ph"] = Po;

  String sensorJsonFormat;
  serializeJson(SensorJDoc, sensorJsonFormat);
  publishSensorData(sensorJsonFormat);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) 
{
  String msgBuff;
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);

  for (int i = 0; i < length; i++) 
  {
    msgBuff += (char) payload[i];
  }

  if (!strcmp(topic, PUMP_TOPIC)) mqttPumpHandler(msgBuff);
}

void monitorWiFiConnection(void* params)
{
  while (true)
  {
    if (WiFi.isConnected()) 
    {
      digitalWrite(LED_BUILTIN, HIGH);
      isWiFiConnected = true;
      wifiManager.stopConfigPortal();
    }
    else
    {
      digitalWrite(LED_BUILTIN, LOW);
      isWiFiConnected = false;
      if (!wifiManager.getConfigPortalActive()) wifiManager.startConfigPortal(AP_SSID, AP_PASSWORD);
    }
    vTaskDelay(250 / portTICK_PERIOD_MS);
  }
}

void connectToMqttBroker(void* params)
{
  while (true)
  {
    if (isWiFiConnected && mqttClient.connected())
    {
      mqttClient.loop();
    }
    else if (isWiFiConnected)
    {
      Serial.println("Attempting to connect MQTT broker");
      if (mqttClient.connect(DEVICE_ID, MQTT_USERNAME, MQTT_PASSWORD))
      {
        Serial.println("Connected to MQTT broker");
        mqttClient.subscribe(PUMP_TOPIC);
      }
      else
      {
        Serial.println("Failed connecting to MQTT broker");
      }
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void mqttPumpHandler(String msg)
{
  EspSerialCom.println(msg);
}

void publishSensorData(String jsonFormattedSensorData) 
{
  if (mqttClient.connected())
  {    
    mqttClient.publish(SENSOR_TOPIC, jsonFormattedSensorData.c_str());
  }
}

int getMedianNum(int bArray[], int iFilterLen) 
{
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++) 
  {
    bTab[i] = bArray[i];
  }
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) 
  {
    for (i = 0; i < iFilterLen - j - 1; i++) 
    {
      if (bTab[i] > bTab[i + 1]) 
      {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0) 
  {
    bTemp = bTab[(iFilterLen - 1) / 2];
  } 
  else 
  {
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  }
  return bTemp;
}