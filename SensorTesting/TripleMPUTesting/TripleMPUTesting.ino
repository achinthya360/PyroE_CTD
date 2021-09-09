#include "MPU9250.h"
#include <SD.h>
#include <SPI.h>

// tail end is mpu (SD0/SC0)
// motor end is mpu 2 (SD2/SC2)

#define TCAADDR 0x70
#define LED 13

// Teensy 3.5 & 3.6 & 4.1 on-board: BUILTIN_SDCARD
const int chipSelect = BUILTIN_SDCARD;
MPU9250 mpu, mpu2, mpu3;

// SD card log file variables
char filename[12];
File dataFile;

// variables for storing mpu data for quicker logging
float t, yaw, pitch, roll, accx, accy, accz;
char data[70];

void setup() {
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  Serial.begin(9600);
  Wire1.begin();
  delay(2000);
  Wire.begin();

  digitalWrite(LED, LOW);
  Serial.println("Beginning SD Card");
  while (!SD.begin(chipSelect));
  updateFilename();

  Serial.println("Beginning Calibration");

  //  tcaselect(0);

  if (!mpu.setup(0x69)) {  // change to your own address
    while (!mpu.setup(0x69)) {
      Serial.println("MPU 1 connection failed. Please check your connection with `connection_check` example.");
      delay(5000);
    }
  }

  mpu.calibrateAccelGyro();

  //  tcaselect(1);
  if (!mpu2.setup(0x68)) {  // change to your own address
    while (!mpu2.setup(0x68)) {
      Serial.println("MPU 2 connection failed. Please check your connection with `connection_check` example.");
      delay(5000);
    }
  }

  mpu2.calibrateAccelGyro();

  //  tcaselect(2);
  if (!mpu3.setup(0x68, MPU9250Setting(), Wire1)) {  // change to your own address
    while (!mpu3.setup(0x68, MPU9250Setting(), Wire1)) {
      Serial.println("MPU 3 connection failed. Please check your connection with `connection_check` example.");
      delay(5000);
    }
  }

  mpu3.calibrateAccelGyro();

  Serial.println("Beginning Data Collection");
  digitalWrite(LED, HIGH);
}

void loop() {
  digitalWrite(LED, !digitalRead(LED));
  // open the file.
  dataFile = SD.open(filename, FILE_WRITE);
  
  //  tcaselect(0);
  if (mpu.update()) {
    static uint32_t prev_ms = millis();
    if (millis() > prev_ms) { // Russell had prev_ms + 100 to buffer the update (probably for decreased power consumption
      print_roll_pitch_yaw();
      prev_ms = millis();
    }
  }

  //  tcaselect(1);
  if (mpu2.update()) {
    static uint32_t prev_ms2 = millis();
    if (millis() > prev_ms2) {
      print2_roll_pitch_yaw();
      prev_ms2 = millis();
    }
  }

  //  tcaselect(2);
  if (mpu3.update()) {
    static uint32_t prev_ms2 = millis();
    if (millis() > prev_ms2) {
      print3_roll_pitch_yaw();
      prev_ms2 = millis();
    }
  }

  // if the file is available, write to it:
  if (dataFile) {
//    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
        Serial.println("Data logged to SD card");
  } else {
    // if the file isn't open, pop up an error:
        Serial.println("error opening datalog.txt");
  }
}

void print_roll_pitch_yaw() {
  t = millis();
  yaw = mpu.getYaw();
  pitch = mpu.getPitch();
  roll = mpu.getRoll();
  accx = mpu.getAccX();
  accy = mpu.getAccY();
  accz = mpu.getAccZ();
  sprintf(data, "1, %.0f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f", t, yaw, pitch, roll, accx, accy, accz);
  Serial.print(data);
  dataFile.print(data);
}

void print2_roll_pitch_yaw() {
  t = millis();
  yaw = mpu2.getYaw();
  pitch = mpu2.getPitch();
  roll = mpu2.getRoll();
  accx = mpu2.getAccX();
  accy = mpu2.getAccY();
  accz = mpu2.getAccZ();
  sprintf(data, ", 2, %.0f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f", t, yaw, pitch, roll, accx, accy, accz);
  Serial.print(data);
  dataFile.print(data);
}

void print3_roll_pitch_yaw() {
  t = millis();
  yaw = mpu3.getYaw();
  pitch = mpu3.getPitch();
  roll = mpu3.getRoll();
  accx = mpu3.getAccX();
  accy = mpu3.getAccY();
  accz = mpu3.getAccZ();
  sprintf(data, ", 3, %.0f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f", t, yaw, pitch, roll, accx, accy, accz);
  Serial.println(data);
  dataFile.println(data);
}

void updateFilename() {
  // Make base filename
  sprintf(filename, "%s000.%s", "LOG", "CSV");
  int namelength = strlen(filename);
  if (namelength > 12) Serial.println("Error: filename too long. Shorten your filename to < 5 characters (12 chars total w number & file extension) !");

  // Keep incrementing the number part of filename until we reach an unused filename
  int i = 1;
  while (SD.exists(filename)) {  // keep looping if filename already exists on card. [If the filename doesn't exist, the loop exits, so we found our first unused filename!]

    int hundreds = i / 100;
    filename[namelength - 7] = '0' + hundreds;
    filename[namelength - 6] = '0' + (i / 10) - (hundreds * 10);
    filename[namelength - 5] = '0' + i % 10;
    i++;

  }
}

//void tcaselect(uint8_t i) {
//  if (i > 7) return;
//  Wire.beginTransmission(TCAADDR);
//  Wire.write(1 << i);
//  Wire.endTransmission();
//}
