#include <Arduino.h>
#include <Adafruit_PM25AQI.h>
#include <SPI.h>

/* Test sketch for Adafruit PM2.5 sensor with UART or I2C */

// If your PM2.5 is UART only, for UNO and others (without hardware serial)
// we must use software serial...
// pin #2 is IN from sensor (TX pin on sensor), leave pin #3 disconnected
// comment these two lines if using hardware serial
//#include <SoftwareSerial.h>
// SoftwareSerial pmSerial(2, 3);

// prototype
// void show_on_lcd(PM25_AQI_Data data);
// void sendToNodeRed(PM25_AQI_Data data);

Adafruit_PM25AQI aqi = Adafruit_PM25AQI();

void setup()
{
  // lol
}
void loop()
{
}
