#include <OneWire.h>
#include <DallasTemperature.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>

SoftwareSerial Esp32SerialCom(2, 3); // rx, tx

const int pinRelay = 7;

// TDS
#define TdsSensorPin A0
#define VREE 5.0  // Sesuaikan tegangan referensi sesuai kebutuhan
#define SCOUNT 30

int analogBuffer[SCOUNT];
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;
int copyIndex = 0;
float averageVoltage = 0;
float tdsValue = 0;

// PH
const int pin_pH = A1;  // Sesuaikan pin analog PH
int pH_analog;
float Po = 0;
float pH_step;
double voltage;
float PH4 = 3.22;
float PH7 = 2.69;

// Temp
const int oneWireBus = 8;
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

void setup() {
  Serial.begin(9600);
  Esp32SerialCom.begin(9600);

  pinMode(TdsSensorPin, INPUT);
  sensors.begin();
  pinMode(pinRelay, OUTPUT);

  // Matikan pompa pada awalnya
  digitalWrite(pinRelay, LOW);
}

void loop() {
  // Temp
  sensors.requestTemperatures();
  float temperatureC = sensors.getTempCByIndex(0);
  Serial.print(temperatureC);
  Serial.println("ºC");

  // TDS
  static unsigned long analogSampleTime = millis();
  if (millis() - analogSampleTime > 40U) {
    analogSampleTime = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT) {
      analogBufferIndex = 0;
    }
  }

  static unsigned long printTimePoint = millis();
  if (millis() - printTimePoint > 800U) {
    printTimePoint = millis();
    for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++) {
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
      averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREE / 1023.0;  // Untuk Arduino Uno, ADC resolution adalah 10 bits (1023)
      float compensationCoefficient = 1.0 + 0.02 * (temperatureC - 25.0);
      float compensationVoltage = averageVoltage / compensationCoefficient;
      tdsValue = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage - 255.86 * compensationVoltage * compensationVoltage + 857.39 * compensationVoltage) * 0.5;
      Serial.print("TDS Value: ");
      Serial.print(tdsValue, 0);
      Serial.println(" ppm");
    }
  }

  // PH
  pH_analog = analogRead(pin_pH);
  Serial.print("ADC Value: ");
  Serial.println(pH_analog);

  // Calculate voltage
  voltage = pH_analog * (5.0 / 1023.0);  // Untuk Arduino Uno, ADC resolution adalah 10 bits (1023)
  Serial.print("Voltage: ");
  Serial.println(voltage, 4);

  // Calculate pH value of the liquid
  pH_step = (PH4 - PH7) / 3;
  Po = 9.4 + ((PH7 - voltage) / pH_step);
  Serial.print("pH Value: ");
  Serial.println(Po, 2);
  Serial.println();

  // Kirim data secara serial
  DynamicJsonDocument doc(JSON_OBJECT_SIZE(3));

  doc["temp"] = temperatureC;
  doc["tds"] = tdsValue;
  doc["ph"] = Po;

  String jsonString;
  serializeJson(doc, jsonString);

  Esp32SerialCom.println(jsonString);

  // digitalWrite(pinRelay, HIGH);
  // Serial.println("Pompa ON");
  // delay(5000); // Delay 5 detik

  // // Matikan pompa selama 5 detik
  // digitalWrite(pinRelay, LOW);
  // Serial.println("Pompa OFF");
  // delay(5000); 
}

int getMedianNum(int bArray[], int iFilterLen) {
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++) {
    bTab[i] = bArray[i];
  }
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0) {
    bTemp = bTab[(iFilterLen - 1) / 2];
  } else {
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  }
  return bTemp;

}