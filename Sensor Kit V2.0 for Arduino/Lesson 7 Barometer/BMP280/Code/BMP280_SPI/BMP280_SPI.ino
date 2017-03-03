/******************************************************
* Name: BMP280_SPI
* Description: Use the SPI interface of BMP280 to detect temperature, air pressure and sea-level height
* Connection:
  BMP280            Uno/MEGA2560
   VCC                   5V
   GND                   GND
   SCK              13 /Pin52 mega2560
   SDO              12 /Pin50 mega2560 
   SDI              11 /Pin51 mega2560
   CS               10 /Pin53 mega2560
*********************************************************/
#include <SPI.h>
#include <Adafruit_BMP280.h>

#define BMP_SCK 13
#define BMP_SDO 12
#define BMP_SDI 11 
#define BMP_CS 10

//Adafruit_BMP280 bmp280(BMP_CS); // hardware SPI
Adafruit_BMP280 bmp280(BMP_CS, BMP_SDI, BMP_SDO,  BMP_SCK); //Define each pin of the SPI port


void setup() {
  Serial.begin(9600);  //Set the baud rate of serial monitor as 9600bps
  Serial.println(F("BMP280_I2C"));//print BMP280_I2C on serial monitor

  if (!bmp280.begin()) //if bmp280.begin()==0, it means bmp280 initialization fails.
  {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);//Infinite loop, no stop until the initialization succeeds.
  }
}

void loop() {
  Serial.print(F("Temperature: ")); 
  Serial.print(bmp280.readTemperature()); //Print the temperature read
  Serial.println(" C"); //Unit of the temperature but ℃ cannot be shown completely

  Serial.print(F("Pressure: "));
  Serial.print(bmp280.readPressure()); //Print the air pressure on serial monitor 
  Serial.println(" Pa");

  Serial.print(F("Approx altitude: ")); //
  Serial.print(bmp280.readAltitude(1013.25)); // Print the altitude. It should be consistent with your local atmospheric pressure.
//  Serial.println(" m");

  Serial.println();
  delay(2000);
}
