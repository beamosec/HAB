#include <SD.h>
#include <Wire.h>
#include <SPI.h>
#include <DHT.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_BMP183.h>

/*
  SD card commmunications pins:
  CS - pin 10 uno 53 mega
  DI - pin 11 uno 51 mega
  DO - pin 12 uno 50 mega
  CLK- pin 13 uno 52 mega
*/
File dataFile;

/*ADXL345commmunications pins:
  A5 - SCL
  A4 - SDA
*/

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

#define BMP183_SCK  13//BMP183 commmunications pins: was CLK
#define BMP183_SDO  12
#define BMP183_SDI  11
#define BMP183_CS   10
#define DHTPIN 2
#define DHTTYPE DHT22
Adafruit_BMP183 bmp = Adafruit_BMP183(BMP183_SCK, BMP183_SDO, BMP183_SDI, BMP183_CS);

/*Blink-LED commmunications pins:
  Anode - pin 9
*/

DHT dht(DHTPIN, DHTTYPE);
int led = 9;

void setup()

{

  Serial.begin(9600); //start serial communication at 9600

  Serial.println("Initializing the SD card...");
  if (!SD.begin(53)) // if statement to test for SD initialization
  {
    Serial.println("SD Initialization failed!");
    return;
  }
  Serial.println("SD Initialization compleate");


  Serial.println("Initializing Accelerometer...");
  if (!accel.begin())
  {
    Serial.println("No Accelerometer detected!");
    return;
  }
  /* Set the range to whatever is appropriate for your project */
  accel.setRange(ADXL345_RANGE_16_G);
  // displaySetRange(ADXL345_RANGE_8_G);
  // displaySetRange(ADXL345_RANGE_4_G);
  // displaySetRange(ADXL345_RANGE_2_G);

  Serial.println("Accelerometer detected");

  Serial.println("Initializing Pressure/Temperature Sensor...");
  if (!bmp.begin())
  {
    Serial.println("No BMP183(Pressure/Temperature) detected ... Check your wiring!");
    return;
  }
  Serial.println("Pressure/Temperature Sensor detected ");

  Serial.println("Initializing Humitidity/Temp Sensor...");
  dht.begin();

  Serial.println("");
  Serial.println("");
  
  pinMode(led, OUTPUT);
}

// Loop to record data to a .csv file every 6 seconds. Program by Nick, Cedric, and Alex
void loop() {
  
  dataFile = SD.open("HawkData.csv", FILE_WRITE); //open the file to write to it

  if (dataFile) {

    /* Get a new sensor event */
    sensors_event_t event;
    accel.getEvent(&event);
    /* Display the results (acceleration is measured in m/s^2) */
    Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
    Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
    Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  "); Serial.println("m/s^2 ");


    /* Display atmospheric pressue in Pascals */
    float pressure;
    pressure = bmp.getPressure();
    Serial.print("Pressure: ");
    Serial.print(pressure);
    Serial.print(" Pascals / ");
    Serial.print(bmp.getPressure() / 100);
    Serial.println(" millibar (hPa)");

    // For SeaLevelPressure, we are currently using generic value of 1013.25 mbar
    // TODO check local sea level pressure and convert it into code
    
    float seaLevelPressure = 1022.9; // should be ~1000
    Serial.print("Sea level pressure: ");
    Serial.print(seaLevelPressure);
    Serial.println(" millibar/hPa");

    float altitude;
    altitude = bmp.getAltitude(seaLevelPressure);
    Serial.print("Altitude: ");
    Serial.print(altitude * 3.28084);
    Serial.println(" ft");

    float humidity = dht.readHumidity();
    float tempaf = dht.readTemperature(true);
    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.print(" humidities");
    Serial.println("");
    Serial.print("Tempature: ");
    Serial.print(tempaf);
    Serial.print(" f");
    Serial.println("");
    Serial.println("");
  
    
    String dataString = String(event.acceleration.x) + ", " + String(event.acceleration.y) + ", " + String(event.acceleration.z);
    
    dataFile.print(millis() / 1000);
    dataFile.print(",");
    dataFile.print(dataString);
    dataFile.print(",");
    dataFile.print(pressure);
    dataFile.print(",");
    dataFile.println(altitude * 3.28084);
    dataFile.print(",");
    dataFile.print(humidity);
    dataFile.print(",");
    dataFile.print(tempaf);

    dataFile.close();
    
  } else {
    
    Serial.println("Error in opening file");
    
    }
    
  delay(2000);
  digitalWrite (led, HIGH);
  delay(2000);
  digitalWrite (led, LOW);
  delay (2000);
} // Delay for next capture currently 6000 miliseconds, can be changed.
