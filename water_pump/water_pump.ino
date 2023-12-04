#include <WiFi.h>
#include <WiFiManager.h>
#include <PubSubClient.h>

void setup() 
{
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  WiFiManager wifiManager;
  wifiManager.setDarkMode(true);
  wifiManager.
  // TODO: Remove this in production
  wifiManager.resetSettings();

  bool res = wifiManager.autoConnect("Hydroponix", "password");
  if (!res) 
  {
    Serial.println("Failed to connect");
  }
  else
  {
    Serial.println("connected yeey :)");
  }
}

void loop() 
{

}
