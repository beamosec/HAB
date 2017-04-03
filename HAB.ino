#include <SD.h>
#include <Wire.h>
#include <SPI.h>
#include <DHT.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_BMP183.h>

#include <SoftwareSerial.h>

File dataFile;

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

#define BMP183_SCK  13 //BMP183 commmunications pins: was CLK
#define BMP183_SDO  12
#define BMP183_SDI  11
#define BMP183_CS   10
#define DHTPIN 2
#define DHTTYPE DHT22
Adafruit_BMP183 bmp = Adafruit_BMP183(BMP183_SCK, BMP183_SDO, BMP183_SDI, BMP183_CS);

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
  Serial.println("SD Initialization complete.");


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
  Serial.println("Humidity/Temp sensor detected!");
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

    // TODO check local sea level pressure for accurate altitude readings
    float seaLevelPressure = 1022.9;
    Serial.print("Sea level pressure: ");
    Serial.print(seaLevelPressure);
    Serial.println(" millibar/hPa");

    float altitude;
    altitude = bmp.getAltitude(seaLevelPressure);
    Serial.print("Altitude: ");
    Serial.print(altitude * 3.28084);
    Serial.println(" ft");

    float humidity = dht.readHumidity();
    float temperature = dht.readTemperature(true);
    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.print(" humidities");
    Serial.println("");
    Serial.print("Tempature: ");
    Serial.print(temperature);
    Serial.print("  °f");
    Serial.println("");

    float heatindex = dht.computeHeatIndex(temperature, humidity);
    Serial.print("Heat index: ");
    Serial.print(heatindex);
    Serial.println(" °f");
    Serial.println("");
  
    
    String dataString = String(event.acceleration.x) + ", " + String(event.acceleration.y) + ", " + String(event.acceleration.z);
    
    dataFile.print(millis() / 1000);
    dataFile.print(",");
    dataFile.print(dataString); //x,y,z
    dataFile.print(",");
    dataFile.print(pressure);
    dataFile.print(",");
    dataFile.println(altitude * 3.28084);
    dataFile.print(",");
    dataFile.print(humidity);
    dataFile.print(",");
    dataFile.print(temperature);
    dataFile.print(",");
    dataFile.print(heatindex);

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
