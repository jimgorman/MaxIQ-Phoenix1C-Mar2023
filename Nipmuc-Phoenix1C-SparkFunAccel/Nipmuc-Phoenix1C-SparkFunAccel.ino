/*
  Program:     Nipmuc-Phoenix1C.ino
  Authors:     Nipmuc Regional High School (Upton, Massachusetts, USA) students & their teacher, Mr. James Gorman
  Date:        Feb 24, 2023
  Description: This script is written for the UKZN ASRI Phoenix 1C 10 km rocket test launch out of Cape Town, South Africa.
               It reads data from sensors such as a barometer and an accelerometer, and writes the data to an SD card. The
               script also adjusts the time delay based on detected acceleration and logs the events to a separate log file.
  Core:        Extended Core - ESP32 Dev Module
  xChips:      Barometer - Goertek SPL06-001 https://github.com/domino4com/IWB
               Accelerometer - STMicroelectronics LIS2DH12 https://github.com/domino4com/IIA
               Weather - Aosong ASAIR AHT21 https://github.com/domino4com/IWA

*/


// libraries needed for working with Extended Core - ESP32 Dev Module
#include <Wire.h>       // communicate with I2C (Inter-Integrated Circuit) devices
#include <SPI.h>        // communicating with devices using the Serial Peripheral Interface (SPI) protocol
#include <SD.h>         // reading and writing data to and from SD (Secure Digital) cards
#include "FS.h"         // interacting with the file system

// libraries needed for the xChips
#include <SPL06-007.h>  // Barometer
#include <AHTxx.h>      // Weather ATH21
#include "SparkFun_LIS2DH12.h" // Accelerometer
SPARKFUN_LIS2DH12 accel;    // creates an instance of the SPARKFUN_LIS2DH12 class
AHTxx aht20(AHTXX_ADDRESS_X38, AHT2x_SENSOR); //sensor address, sensor type


// defines constant values for various pins to be used on the Extended Core
#define I2C_SDA 26    // pin number for the data line of an I2C bus
#define I2C_SCL 27    // in number for the clock line of an I2C bus
#define SPI_MISO 12   // pin number for the MISO (Master In, Slave Out) line of a SPI bus - SD Card
#define SPI_MOSI 13   // pin number for the MOSI (Master Out, Slave In) line of a SPI bus - SD Card
#define SPI_SCK 14    // pin number for the SCK (Serial Clock) line of a SPI bus - SD Card
#define SD_CS 5       // pin number for the chip select (CS) line of an SD card module  - SD Card


// Declare global variables          
const String dataFileName = "/data.txt";   // variable for the data file name
double local_pressure = 1000.9; // lowest pressure in Cape Town over next few days.
unsigned long t;         // store the time
float accelX;       // store acceleration in the x dimension
float accelY;       // store acceleration in the y dimension
float accelZ;       // store acceleration in the z dimension
float accelVectorMagnitude;    // acceleration vector magnitude
double alt;         // store altitude calculation
double press;       // store pressure
double temp;        // store temperature
float relHumidity;         // store relative humidity
bool Accerror = false;  //
bool Barrerror = false; //
bool Weather = false;   //
bool SDerror = false;   //
String data = "";       // global string variable for storing measurement to be stored in file on SD card
int dLONG = 2000;   // in milliseconds. 2000 ms = 2 s
int dSHORT = 50;    // in milliseconds. 500 ms = 0.5 s
int D = dLONG;             // time delay variable
int rocketMovement = 1.5;     // threshold for shortening the delay


// Add new data to the end of the data file
void appendFile(fs::FS &fs, const char * path, const char * message){
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if(!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
 
  if(file.print(message)) {
    Serial.println("Message appended");
    } else {
      Serial.println("Append failed");
  }
 
  file.flush();
  file.close();
}


// Puts measurements into a string and send it off to be added to the data file
void logData(){
  t = millis();
  
  if (accel.available())
    {
      accelX = accel.getX()*0.001;  // reading is in mg the 0.001 converts to g
      accelY = accel.getY()*0.001;
      accelZ = accel.getZ()*0.001;
    } else {
        accelX = 0;
        accelY = 0;
        accelZ = 0;
  }
 
  // Calculate the magnitude of the acceleration vector
  accelVectorMagnitude = sqrt(accelX*accelX + accelY*accelY + accelZ*accelZ);

  // adjust sampling rate based on acceleration vector
  if(D == dLONG && accelVectorMagnitude > rocketMovement) {
    D = dSHORT;  // shorten delay
    
    //report to change to log
    String txt = String(t) + "  🚀 Significant Acceleration (" + String(accelVectorMagnitude) + ") - Shortening the delay to " + D + " ms\n";
    appendFile(SD, "/log.txt", txt.c_str());
    
  } else if (D == dSHORT && accelVectorMagnitude < rocketMovement){
    D = dLONG;  // lengthen delay
    
    //report to change to log
    String txt = String(t) + "  inSignificant Acceleration (" + String(accelVectorMagnitude) + ") - Lengthening the delay to " + D + " ms\n";
    appendFile(SD, "/log.txt", txt.c_str());
  }
 
  press = get_pressure();
  alt = get_altitude(press,local_pressure);
  data = "";   // reset data string

  // gather gather the measurements and add to data string
  // timestamp
  data.concat(String(t));
  data.concat(",");
  // Accelerometer xChip
  data.concat(String(accelX));
  data.concat(",");
  data.concat(String(accelY));
  data.concat(",");
  data.concat(String(accelZ));
  data.concat(",");
  data.concat(String(accelVectorMagnitude));
  data.concat(",");
  // barometer xChip
  data.concat(String(get_temp_c()));
  data.concat(",");
  data.concat(String(press));
  data.concat(",");
  data.concat(String(alt));
  // weather xChip
  data.concat(",");
  data.concat(String(aht20.readHumidity()));   // take 80 ms to read the sensor
  data.concat("\n");

  //add data string to file on SD card
  appendFile(SD, dataFileName.c_str(), data.c_str());  
}


void setup(){
  Serial.begin(115200);   // initializes the Serial communication interface with a baud rate of 115200 bits per second
  Wire.setPins(I2C_SDA, I2C_SCL);   // sets the pins for the I2C communication interface with Extended Core
  Wire.begin();   // initializes the I2C communication interface with Extended Core

  // initializes the Serial Peripheral Interface (SPI) communication protocol for an SD card and attempts to mount the SD card on the SPI bus
  SPIClass spi = SPIClass(HSPI);
  spi.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SD_CS);
  if (!SD.begin(SD_CS, spi,80000000)) {
    Serial.println(F("Card Mount Failed"));
    String txt = String(millis()) + "  Card Mount Failed\n";
    appendFile(SD, "/log.txt", txt.c_str());
    SDerror = true;
  }

  // determine there is an SD card
  uint8_t cardType = SD.cardType();
  if(cardType == CARD_NONE){
      SDerror = true;
      String txt = String(millis()) + "  No SD card attached\n";
      appendFile(SD, "/log.txt", txt.c_str());
  }

  // Add the headers to data CSV file
  appendFile(SD, "/data.txt", "time(ms),accelX(g),accelY(g),accelZ(g),accelVector(g),temp(C),press(mb),alt(m),rh(%)\n");
  
  // Add start time to log file
  String txt = String(millis()) + "  START...\n";
  appendFile(SD, "/log.txt", txt.c_str());

  // Initializes the communication with the digital barometric pressure and temperature sensor module with the I2C interface
  SPL_init(0x77);
 
  // check to make sure accelerometer is working
  if (accel.begin() == false)
  {
    //Serial.println(F("Accelerometer not detected"));
    txt = String(millis()) + "  Accelerometer not detected\n";
    appendFile(SD, "/log.txt", txt.c_str());
    Accerror = true;
  }

  // set up accelerometer
  accel.setScale(LIS2DH12_16g);
  accel.setMode(LIS2DH12_HR_12bit);
  accel.setDataRate(LIS2DH12_ODR_400Hz);
  while (aht20.begin() != true) //for ESP-01 use aht20.begin(0, 2);
  {
    Serial.println(F("AHT2x not connected or fail to load calibration coefficient")); //(F()) save string to flash & keeps dynamic memory free
    appendFile(SD, "/log.txt", "AHT2x not connected or fail to load calibration coefficient\n");
    delay(5000);
  }
  Serial.println(F("AHT20 OK"));
  
  // Log delay period
  txt = String(millis()) + "  Initial reading delay setting is " + D + " ms\n";
  appendFile(SD, "/log.txt", txt.c_str());

  logData();   // get & store the measurements
  delay(D);    // delay to prevent oversampling which leads to sensor overheating & save energy

 // Since setup keeps restarting and not going to the loop() we put loop() into this while statement
  while (1 == 1) {
    // end the code execution if there is a problem with one of the sensors
    if(Accerror){
      String txt = String(millis()) + "  accel error\n";
      appendFile(SD, "/log.txt", txt.c_str());  
    } else if(SDerror){
       String txt = String(millis()) + "  SD error\n";
       appendFile(SD, "/log.txt", txt.c_str());
    } else if(Barrerror){
       String txt = String(millis()) + "  barr error\n";
       appendFile(SD, "/log.txt", txt.c_str()); 
    }

    logData();   // get & store the measurements
    delay(D);    // delay to prevent oversampling which leads to sensor overheating & save energy
  }  
}


void loop() {
/* commented out this section because the script would not transition successfully to the loop it 
  appendFile(SD, dataFileName.c_str(), "Made it to the loop!\n");  
  // end the code execution if there is a problem with one of the sensors
  if(Accerror){
       String txt = String(millis()) + "  accel error\n";
       appendFile(SD, "/log.txt", txt.c_str());  
   }else if(SDerror){
       String txt = String(millis()) + "  SD error\n";
       appendFile(SD, "/log.txt", txt.c_str());
   }else if(Barrerror){
       String txt = String(millis()) + "  barr error\n";
       appendFile(SD, "/log.txt", txt.c_str());
  }

  logData();
  delay(D);    // delay to prevent oversampling which leads to sensor overheating & save energy
*/
}