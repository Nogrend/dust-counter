#include <Arduino.h>
#include <Adafruit_PM25AQI.h>
#include <SPI.h>
#include <LiquidCrystal_I2C.h>

/* Test sketch for Adafruit PM2.5 sensor with UART or I2C */

// If your PM2.5 is UART only, for UNO and others (without hardware serial)
// we must use software serial...
// pin #2 is IN from sensor (TX pin on sensor), leave pin #3 disconnected
// comment these two lines if using hardware serial
//#include <SoftwareSerial.h>
// SoftwareSerial pmSerial(2, 3);

// prototype
void show_on_lcd(PM25_AQI_Data data);
void sendToNodeRed(PM25_AQI_Data data);

Adafruit_PM25AQI aqi = Adafruit_PM25AQI();

LiquidCrystal_I2C lcd(0x27, 20, 4); // set the LCD address to 0x27 for a 16 chars and 2 line display

void setup()
{

  // Wait for serial monitor to open
  Serial.begin(9600);
  while (!Serial)
    delay(10);

  Serial.println("Adafruit PMSA003I Air Quality Sensor");

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);

  // Wait one second for sensor to boot up!
  delay(1000);

  // If using serial, initialize it and set baudrate before starting!
  // Uncomment one of the following
  // Serial1.begin(9600);
  // pmSerial.begin(9600);

  // There are 3 options for connectivity!
  if (!aqi.begin_I2C())
  { // connect to the sensor over I2C
    // if (! aqi.begin_UART(&Serial1)) { // connect to the sensor over hardware serial
    // if (! aqi.begin_UART(&pmSerial)) { // connect to the sensor over software serial
    Serial.println("Could not find PM 2.5 sensor!");
    while (1)
      delay(10);
  }

  Serial.println("PM25 found!");
}

void loop()
{
  PM25_AQI_Data data;

  if (!aqi.read(&data))
  {
    Serial.println("Could not read from AQI");
    delay(500); // try again in a bit!
    return;
  }
  // Serial.println("AQI reading success");

  // Serial.println();
  // Serial.println(F("---------------------------------------"));
  // Serial.println(F("Concentration Units (standard)"));
  // Serial.println(F("---------------------------------------"));
  // Serial.print(F("PM 1.0: "));
  // Serial.print(data.pm10_standard);
  // Serial.print(F("\t\tPM 2.5: "));
  // Serial.print(data.pm25_standard);
  // Serial.print(F("\t\tPM 10: "));
  // Serial.println(data.pm100_standard);
  // Serial.println(F("Concentration Units (environmental)"));
  // Serial.println(F("---------------------------------------"));
  // Serial.print(F("PM 1.0: "));
  // Serial.print(data.pm10_env);
  // Serial.print(F("\t\tPM 2.5: "));
  // Serial.print(data.pm25_env);
  // Serial.print(F("\t\tPM 10: "));
  // Serial.println(data.pm100_env);
  // Serial.println(F("---------------------------------------"));
  // Serial.print(F("Particles > 0.3um / 0.1L air:"));
  // Serial.println(data.particles_03um);
  // Serial.print(F("Particles > 0.5um / 0.1L air:"));
  // Serial.println(data.particles_05um);
  // Serial.print(F("Particles > 1.0um / 0.1L air:"));
  // Serial.println(data.particles_10um);
  // Serial.print(F("Particles > 2.5um / 0.1L air:"));
  // Serial.println(data.particles_25um);
  // Serial.print(F("Particles > 5.0um / 0.1L air:"));
  // Serial.println(data.particles_50um);
  // Serial.print(F("Particles > 10 um / 0.1L air:"));
  // Serial.println(data.particles_100um);
  // Serial.println(F("---------------------------------------"));
  sendToNodeRed(data);
  show_on_lcd(data);

  delay(1000);
}

void sendToNodeRed(PM25_AQI_Data data)
{

  String dataString = String(data.particles_03um) + "," + String(data.particles_05um) + "," + String(data.particles_10um) + "," + String(data.particles_25um) + "," + String(data.particles_50um) + "," + String(data.particles_100um);
  Serial.println(dataString);
}

void show_on_lcd(PM25_AQI_Data data)
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("P0.3 ");
  lcd.print(data.particles_03um);
  lcd.setCursor(10, 0);
  lcd.print("P0.5 ");
  lcd.print(data.particles_05um);
  lcd.setCursor(0, 1);
  lcd.print("P1.0 ");
  lcd.print(data.particles_10um);
  lcd.setCursor(10, 1);
  lcd.print("P2.5 ");
  lcd.print(data.particles_25um);
  lcd.setCursor(0, 2);
  lcd.print("P5.0 ");
  lcd.print(data.particles_50um);
  lcd.setCursor(10, 2);
  lcd.print("P 10 ");
  lcd.print(data.particles_100um);
}