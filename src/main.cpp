#include <Arduino.h>
#include <Adafruit_PM25AQI.h>
#include <SPI.h>
#include <SD.h>

#define OFF 0
#define GREEN 1
#define RED 2

const int8_t buttonPin = 3;
const int8_t ledPinAnode = 4;
const int8_t ledPinCathode = 5;
const int8_t chipSelect = 10;

Adafruit_PM25AQI aqi = Adafruit_PM25AQI();

// prototypes
void setLedIndicator(uint8_t);
bool isButtonClosed(void);
void getAirQualityReading(PM25_AQI_Data *pointerData);
void sendAirQualityDataOverSerial(PM25_AQI_Data);
void saveAirQualityDataOnSd(PM25_AQI_Data);

void setup()
{
  // initialization serial communication
  Serial.begin(9600);

  // initialization sd card writer/reader
  if (!SD.begin(chipSelect))
    while (1)
      delay(10);

  // initialization air quality sensor communication
  if (!aqi.begin_I2C())
    while (1)
      delay(10);

  // initialization button
  pinMode(buttonPin, INPUT);

  // initialization led indicator
  pinMode(ledPinAnode, OUTPUT);
  digitalWrite(ledPinAnode, LOW);
  pinMode(ledPinCathode, OUTPUT);
  digitalWrite(ledPinCathode, LOW);
}

void loop()
{
  if (isButtonClosed())
  {
    setLedIndicator(GREEN);

    PM25_AQI_Data data;
    getAirQualityReading(&data);

    setLedIndicator(RED);
    sendAirQualityDataOverSerial(data);
    saveAirQualityDataOnSd(data);
    setLedIndicator(GREEN);
  }
  else
  {
    setLedIndicator(OFF);
  }
  delay(5000);
}

void setLedIndicator(uint8_t state)
{
  switch (state)
  {
  case OFF:
    digitalWrite(ledPinAnode, LOW);
    digitalWrite(ledPinCathode, LOW);
    break;

  case GREEN:
    digitalWrite(ledPinAnode, HIGH);
    digitalWrite(ledPinCathode, LOW);
    break;

  case RED:
    digitalWrite(ledPinAnode, LOW);
    digitalWrite(ledPinCathode, HIGH);
    break;

  default:
    break;
  }
}

bool isButtonClosed(void)
{
  bool buttonState = false;
  if (!digitalRead(buttonPin))
  {
    buttonState = true;
  }
  delay(500);
  return buttonState;
}

void getAirQualityReading(PM25_AQI_Data *pointerData)
{
  aqi.read(pointerData);
}

void sendAirQualityDataOverSerial(PM25_AQI_Data data)
{
  String dataString = String(data.particles_03um) +
                      "," + String(data.particles_05um) +
                      "," + String(data.particles_10um) +
                      "," + String(data.particles_25um) +
                      "," + String(data.particles_50um) +
                      "," + String(data.particles_100um);
  Serial.println(dataString);
}

void saveAirQualityDataOnSd(PM25_AQI_Data data)
{
  String dataString = String(data.particles_03um) +
                      "," + String(data.particles_05um) +
                      "," + String(data.particles_10um) +
                      "," + String(data.particles_25um) +
                      "," + String(data.particles_50um) +
                      "," + String(data.particles_100um);
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  dataFile.println(dataString);
  dataFile.close();
}