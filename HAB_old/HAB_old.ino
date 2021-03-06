#include <SD.h> 
#include <Wire.h> 
#include <SPI.h> 
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
 
 
#define BMP183_SCK  8//BMP183 commmunications pins: was CLK 
#define BMP183_SDO  7   
#define BMP183_SDI  6    
#define BMP183_CS   5 
Adafruit_BMP183 bmp = Adafruit_BMP183(BMP183_SCK, BMP183_SDO, BMP183_SDI, BMP183_CS); 
 
 
/*Blink-LED commmunications pins: 
Anode - pin 9 
*/ 
int led=9; 
 
 
long seconds = millis()/1000; 
 
 
void setup() 
 
 
{ 
 
 
  Serial.begin(9600); //start serial communication at 9600 
 
 
Serial.println("Initializing the SD card..."); 
if (!SD.begin(53)) // if statement to test for SD initialization 
{ 
  Serial.println("SD Initialization failed!!!"); 
  return; 
} 
Serial.println("SD Initialization compleate"); 
Serial.println(""); 
 
 
 
 
Serial.println("Initializing Accelerometer..."); 
if(!accel.begin()) 
{ 
  Serial.println("No Accelerometer detected!!!"); 
  return; 
} 
 /* Set the range to whatever is appropriate for your project */ 
  accel.setRange(ADXL345_RANGE_16_G); 
  // displaySetRange(ADXL345_RANGE_8_G); 
  // displaySetRange(ADXL345_RANGE_4_G); 
  // displaySetRange(ADXL345_RANGE_2_G); 
   
  Serial.println("Accelerometer detected"); 
 Serial.println(""); 
   
   
 Serial.println("Initializing Pressure/Temperature Sensor..."); 
 if(!bmp.begin()) 
  { 
    Serial.println("No BMP183 detected ... Check your wiring!!!"); 
    return; 
  } 
  Serial.println("Pressure/Temperature Sensor detected "); 
  Serial.println(""); 
  Serial.println(""); 
   
  pinMode(led, OUTPUT); 
 
 
} 
 
 
 
 
void loop() 
{ 
dataFile = SD.open("HawkData.csv", FILE_WRITE); //open the file to write to it 
 
 
if (dataFile) 
{ 
   
   /* Get a new sensor event */  
  sensors_event_t event;  
  accel.getEvent(&event); 
  /* Display the results (acceleration is measured in m/s^2) */ 
  Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  "); 
  Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  "); 
  Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 "); 
   
   
   /* Display atmospheric pressue in Pascals */ 
    float pressure; 
    pressure = bmp.getPressure(); 
    Serial.print("Pressure:    "); 
    Serial.print(pressure); 
    Serial.print(" Pascals / "); 
    Serial.print(bmp.getPressure() / 100); 
    Serial.println(" millibar (hPa)"); 
 
 
    /* First we get the current temperature from the BMP085 */ 
    float temperature; 
    temperature = bmp.getTemperature(); 
    Serial.print("Temperature: "); 
    Serial.print(temperature); 
    Serial.println(" F"); 
     
    /* Calculating altitude with reasonable accuracy requires pressure    * 
     * sea level pressure for your position at the moment the data is     * 
     * converted. If you don't have these values, a 'generic' value of    * 
     * 1013.25 mbar can be used (defined as SENSORS_PRESSURE_SEALEVELHPA  * 
     * in sensors.h), but this isn't ideal and will give variable         * 
     * results from one day to the next.                                  * 
     *                                                                    * 
     * You can usually find the current SLP value by looking at weather   * 
     * websites or from environmental information centers near any major  * 
     * airport.                                                           * 
     *                                                                    * 
     * For example, for Paris, France you can check the current mean      * 
     * pressure and sea level at: http://bit.ly/16Au8ol                   */ 
      
 
 
    /* Then convert the atmospheric pressure, SLP and temp to altitude    */ 
    /* Update this next line with the current SLP for better results      */ 
    float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA; // should be ~1000 
    Serial.print("Sea level pressure: ");  
    Serial.print(SENSORS_PRESSURE_SEALEVELHPA); 
    Serial.println(" millibar/hPa"); 
     
    float altitude; 
    altitude = bmp.getAltitude(seaLevelPressure); 
    Serial.print("Altitude:    ");  
    Serial.print(altitude);  
    Serial.println(" m"); 
    Serial.println(""); 
     
   
String dataString = String(event.acceleration.x)+ ", " + String(event.acceleration.y)+ ", " + String(event.acceleration.z); 
dataFile.print(millis()/1000); 
dataFile.print(","); 
dataFile.print(dataString); 
dataFile.print(","); 
dataFile.print(pressure); 
dataFile.print(","); 
dataFile.print(temperature*1.8+32); 
dataFile.print(","); 
dataFile.println(altitude); 
 
 
dataFile.close(); 
} 
else 
{ 
Serial.println("Error in opening file"); 
} 
delay(2000); 
digitalWrite (led, HIGH); 
  delay(2000); 
  digitalWrite (led, LOW); 
  delay (2000); 
} 
 
