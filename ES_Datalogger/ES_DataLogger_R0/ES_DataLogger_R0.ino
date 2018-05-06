#include <SparkFunMPU9250-DMP.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "SparkFunMPL3115A2.h"


#define SerialPort Serial
#define RLED 13

/* Things to add: 
 *  Don't wait for Serial on startup
 *  Some kind of filtering options
 *  Some kind of sensor fusion options
 *  Optimize code timing (current cycle is 20-40ms
 *  Speed up SD card communication
 *  Possibly add better USB support with computer
 * 
 *  What this code does:
 *  Logs data from sensors on to SD card
 */

//Sensors
MPU9250_DMP imu;
MPL3115A2 altimeter;

//SD Card variables
File dataFile;
const int SD_CS = 4;
String filename = "log_";
String data;
unsigned long dataTime;

//Timing variables
unsigned long start;
unsigned long end_time;

//Acclerometer Variables
float accelX;
float accelY;
float accelZ;
float gyroX;
float gyroY;
float gyroZ;
float altitude;
float temp;

void createFile(){
/* This function creates a file on the SD card. 
 * Each file is named log_#, where the # correseponds to the number
 * of log files on the SD card. Each file is saved as a .csv.
 */
  boolean fileCreated = false;
  int fileNum = 0;
  while(!fileCreated){
    if(SD.exists(filename + fileNum + ".csv")){
      fileNum++;
    }else{
      filename = filename + fileNum + ".csv";
      dataFile = SD.open(filename, FILE_WRITE);
      if(dataFile){
        dataFile.println(F("Time, Alt, Temp, accelX, accelY, accelZ, gyroX, gyroY, gyroZ"));
      }else{
        Serial.println("Could not write to SD");
        digitalWrite(RLED, HIGH);
      }
      
      fileCreated = true;
    }
  }
  dataFile.close();
  Serial.println("File " + filename + " created");
}

void writeData(){
 /* Writes the sensor data to the SD card
  * Currently takes around 12-25 ms
  */ 
  dataFile = SD.open(filename, FILE_WRITE);
  if(dataFile){
    data = String(dataTime,DEC) + ", " + String(altitude,2) + ", " + String(temp,3) + ", " + String(accelX,4) + ", " + String(accelY,4) + ", " + String(accelZ,4)
     + ", " + String(gyroX,4) + ", " + String(gyroY,4) + ", " + String(gyroZ,4);
    dataFile.println(data);
    dataFile.close();
    //Serial.println("Data Logged");
  }else{
    Serial.println("File not opened");
    digitalWrite(RLED, HIGH);
  }
}

void getData(){
  /* Gets data from all the sensors and stores it in the sensor variables
   * 
   */
  //Get IMU Data
  if (imu.dataReady())
  {
    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    imu.update(UPDATE_ACCEL | UPDATE_GYRO | UPDATE_COMPASS);
    accelX = imu.calcAccel(imu.ax);
    accelY = imu.calcAccel(imu.ay);
    accelZ = imu.calcAccel(imu.az);
    gyroX = imu.calcGyro(imu.gx);
    gyroY = imu.calcGyro(imu.gy);
    gyroZ = imu.calcGyro(imu.gz);
  }
  //Get altimeter data
  altitude = altimeter.readAltitudeFt();
  temp = altimeter.readTempF();
  dataTime = millis();
}

void initSensors(){
  /* Initalizes all the sensors
   * 
   */
  //Intialize MPU 9250
  if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {
      SerialPort.println("Unable to communicate with MPU-9250");
      SerialPort.println("Check connections, and try again.");
      SerialPort.println();
      delay(5000);
    }
  }
  imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);
  // Use setGyroFSR() and setAccelFSR() to configure the
  // gyroscope and accelerometer full scale ranges.
  // Gyro options are +/- 250, 500, 1000, or 2000 dps
  imu.setGyroFSR(2000); // Set gyro to 2000 dps
  // Accel options are +/- 2, 4, 8, or 16 g
  imu.setAccelFSR(16); // Set accel to +/-2g
  // setLPF() can be used to set the digital low-pass filter
  // of the accelerometer and gyroscope.
  // Can be any of the following: 188, 98, 42, 20, 10, 5
  // (values are in Hz).
  imu.setLPF(50); // Set LPF corner frequency to 5Hz
  // The sample rate of the accel/gyro can be set using
  // setSampleRate. Acceptable values range from 4Hz to 1kHz
  imu.setSampleRate(500); // Set sample rate to 10Hz
  imu.setCompassSampleRate(10); // Set mag rate to 10Hz
  
  //Initalize MPL3115A2 altimeter
  altimeter.begin();
  altimeter.setModeAltimeter();
  altimeter.setOversampleRate(3); //Takes values 0-7, higher values cause much higher latency (7 takes 500ms per reading)
  altimeter.enableEventFlags();
}
void setup() {
  pinMode(RLED,OUTPUT); //Intialize LED
  SerialPort.begin(115200);
  while(!Serial){ //Wait for USB connection

  }
  Serial.print("Initializing SD card...");
  if (!SD.begin(SD_CS)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");
  createFile();
  initSensors();
}

void loop() {
    start = millis();
    getData();
    writeData();
    end_time = millis();
    end_time = end_time - start;
    Serial.println(end_time);
    if(!(end_time > 50)){
      delay(50-end_time);
    }
}
