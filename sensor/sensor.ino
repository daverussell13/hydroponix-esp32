#include <WiFi.h>
#include <esp_now.h>

typedef struct {
  float ph;
  float tds;
  float temp;
} Sensor;

uint8_t broadcastAddress[] = {0xB0, 0xA7, 0x32, 0x2B, 0x0A, 0xB0};

// Globals
Sensor sensor;
esp_now_peer_info_t peerInfo;

// Functions
void espNowSendCallback(const uint8_t*, esp_now_send_status_t);

void setup() 
{
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  if(esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(espNowSendCallback);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if(esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {
  sensor.ph = 7.77;
  sensor.tds = 130;
  sensor.temp = 30.7;

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*) &sensor, sizeof(sensor));
  if(result == ESP_OK)
  {
    Serial.println("Sending Confirmed");
  }
  else
  {
    Serial.println("Sending error");
  }
  delay(2000);
}

void espNowSendCallback(const uint8_t* mac_addr, esp_now_send_status_t status)
{
  Serial.print("Last Packet Send Status : ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

