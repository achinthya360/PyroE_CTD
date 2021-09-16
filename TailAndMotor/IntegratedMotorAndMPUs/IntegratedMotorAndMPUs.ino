// Synchronous tail motor control and MPU data collection sketch

// motor control variables/definitions
#define kinematic_coeff 3.35
const float max_speed = 5000; // maximum possible ~ 5000

float microsteps = 800; // from driver
float theta_max = 30; // degrees
long amplitude = microsteps / 360 * theta_max; // microsteps needed to reach wanted angle
//float accel = 90000;  // maximum possible ~ 85000
float freq_wanted = 1;

// theoretical calculations assume constant linear stepping acceleration
//float freq = 3.35 * sqrt(accel/(theta_max * microsteps)); // could be used to back calculate accel
float a = pow(freq_wanted*sqrt(theta_max*microsteps) / kinematic_coeff, 2); // a = (f * sqrt(theta * microsteps) / 3.35) ^ 2

#include <AccelStepper.h>

// Define a stepper and the pins it will use
AccelStepper stepper(AccelStepper::DRIVER, 3, 2); // Using stepper pins (3 for pulse, 2 for dir)



// MPU data collection variables/definitions
#include "MPU9250.h"
#include <SD.h>
#include <SPI.h>

// tail end is mpu (SD0/SC0)
// motor end is mpu 2 (SD2/SC2)

// AD0 connections:
// D2: mpu 2
// D3: mpu 1
// D6: mpu

#define LED LED_BUILTIN
#define AD0_0 6
#define AD0_1 3
#define AD0_2 2

// Teensy 3.5 & 3.6 & 4.1 on-board: BUILTIN_SDCARD
// Arduino Nano Every: 10
const int chipSelect = 10;
MPU9250 mpu, mpu2, mpu3;

// SD card log file variables
char filename[12];
File dataFile;

// variables for storing mpu data for quicker logging
unsigned long t;
int pos;
float yaw, pitch, roll, accx, accy, accz, angle;
char data[70];



void setup() {
  // motor setup
  stepper.setMaxSpeed(max_speed);
  stepper.setAcceleration(a);

  // following pins 4 and 5 could also just be GNDed
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);

  // MPU setup
  pinMode(AD0_0, OUTPUT);
  pinMode(AD0_1, OUTPUT);
  pinMode(AD0_2, OUTPUT);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  Serial.begin(115200);
  delay(2000);
  Wire.begin();

  digitalWrite(LED, LOW);
  Serial.println("Beginning SD Card");
  while (!SD.begin(chipSelect));
  updateFilename();

  Serial.println("Beginning Calibration");

  digitalWrite(LED, HIGH);
  mpuselect(0);
  Serial.println("MPU 1 selected!");

  if (!mpu.setup(0x68)) {  // change to your own address
    while (!mpu.setup(0x68)) {
      Serial.println("MPU 1 connection failed. Please check your connection with `connection_check` example.");
      delay(5000);
    }
  }
  Serial.println("MPU 1 connected, calibrating...");
  mpu.calibrateAccelGyro();

  mpuselect(1);
  if (!mpu2.setup(0x68)) {  // change to your own address
    while (!mpu2.setup(0x68)) {
      Serial.println("MPU 2 connection failed. Please check your connection with `connection_check` example.");
      delay(5000);
    }
  }

  mpu2.calibrateAccelGyro();

  mpuselect(2);
  if (!mpu3.setup(0x68)) {  // change to your own address
    while (!mpu3.setup(0x68)) {
      Serial.println("MPU 3 connection failed. Please check your connection with `connection_check` example.");
      delay(5000);
    }
  }

  mpu3.calibrateAccelGyro();

  Serial.println("Beginning Data Collection");
  digitalWrite(LED, HIGH);

  // initiate first sweep of motor
  stepper.moveTo(amplitude);
}

void loop() {
  // Motor movement functionality
  // If at the end of travel go to the other end
  if (stepper.distanceToGo() == 0)
    stepper.moveTo(-stepper.currentPosition());
  stepper.run();


  // open the SD card data file
  dataFile = SD.open(filename, FILE_WRITE);

  mpuselect(1);
  if (mpu2.update()) {

  }

  // flick status LED
  digitalWrite(LED, !digitalRead(LED));

  mpuselect(0);
  if (mpu.update()) {
    static uint32_t prev_ms = millis();
    if (millis() > prev_ms) { // Russell had prev_ms + 100 to buffer the update (probably for decreased power consumption)
      print_roll_pitch_yaw();
      prev_ms = millis();
    }
  }

  mpuselect(1);
  if (mpu2.update()) {
    static uint32_t prev_ms2 = millis();
    if (millis() > prev_ms2) {
      print2_roll_pitch_yaw();
      prev_ms2 = millis();
    }
  }

  mpuselect(2);
  if (mpu3.update()) {
    static uint32_t prev_ms3 = millis();
    if (millis() > prev_ms3) {
      print3_roll_pitch_yaw();
      prev_ms3 = millis();
    }
  }
  
  // output motor position and angle values
  pos = stepper.currentPosition();
  angle = pos / microsteps * 360;
  Serial.print(", ");
  Serial.print(pos);
  Serial.print(", ");
  Serial.println(angle);
  // if the file is available, write to it:
  if (dataFile) {
    dataFile.print(", ");
    dataFile.print(pos);
    dataFile.print(", ");
    dataFile.println(angle);
    dataFile.close();
  } else {
    //    // if the file isn't open, pop up an error:
    //    Serial.println("error opening datalog.txt");
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
  Serial.print("1, ");
  Serial.print(t);
  Serial.print(", ");
  Serial.print(yaw);
  Serial.print(", ");
  Serial.print(pitch);
  Serial.print(", ");
  Serial.print(roll);
  Serial.print(", ");
  Serial.print(accx);
  Serial.print(", ");
  Serial.print(accy);
  Serial.print(", ");
  Serial.print(accz);
  //  printf(data, "1, %d, %f, %f, %f, %f, %f, %f", t, yaw, pitch, roll, accx, accy, accz);
  //  Serial.print(data);
  //  dataFile.print(data);
  dataFile.print("1, ");
  dataFile.print(t);
  dataFile.print(", ");
  dataFile.print(yaw);
  dataFile.print(", ");
  dataFile.print(pitch);
  dataFile.print(", ");
  dataFile.print(roll);
  dataFile.print(", ");
  dataFile.print(accx);
  dataFile.print(", ");
  dataFile.print(accy);
  dataFile.print(", ");
  dataFile.print(accz);
}

void print2_roll_pitch_yaw() {
  t = millis();
  yaw = mpu2.getYaw();
  pitch = mpu2.getPitch();
  roll = mpu2.getRoll();
  accx = mpu2.getAccX();
  accy = mpu2.getAccY();
  accz = mpu2.getAccZ();
  Serial.print(", 2, ");
  Serial.print(t);
  Serial.print(", ");
  Serial.print(yaw);
  Serial.print(", ");
  Serial.print(pitch);
  Serial.print(", ");
  Serial.print(roll);
  Serial.print(", ");
  Serial.print(accx);
  Serial.print(", ");
  Serial.print(accy);
  Serial.print(", ");
  Serial.print(accz);
  //  sprintf(data, ", 2, %.0f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f", t, yaw, pitch, roll, accx, accy, accz);
  //  Serial.print(data);
  //  dataFile.print(data);
  dataFile.print(", 2, ");
  dataFile.print(t);
  dataFile.print(", ");
  dataFile.print(yaw);
  dataFile.print(", ");
  dataFile.print(pitch);
  dataFile.print(", ");
  dataFile.print(roll);
  dataFile.print(", ");
  dataFile.print(accx);
  dataFile.print(", ");
  dataFile.print(accy);
  dataFile.print(", ");
  dataFile.print(accz);
}

void print3_roll_pitch_yaw() {
  t = millis();
  yaw = mpu3.getYaw();
  pitch = mpu3.getPitch();
  roll = mpu3.getRoll();
  accx = mpu3.getAccX();
  accy = mpu3.getAccY();
  accz = mpu3.getAccZ();
  Serial.print(", 3, ");
  Serial.print(t);
  Serial.print(", ");
  Serial.print(yaw);
  Serial.print(", ");
  Serial.print(pitch);
  Serial.print(", ");
  Serial.print(roll);
  Serial.print(", ");
  Serial.print(accx);
  Serial.print(", ");
  Serial.print(accy);
  Serial.print(", ");
  Serial.print(accz);
  //  sprintf(data, ", 3, %.0f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f", t, yaw, pitch, roll, accx, accy, accz);
  //  Serial.println(data);
  //  dataFile.println(data);
  dataFile.print(", 3, ");
  dataFile.print(t);
  dataFile.print(", ");
  dataFile.print(yaw);
  dataFile.print(", ");
  dataFile.print(pitch);
  dataFile.print(", ");
  dataFile.print(roll);
  dataFile.print(", ");
  dataFile.print(accx);
  dataFile.print(", ");
  dataFile.print(accy);
  dataFile.print(", ");
  dataFile.print(accz);
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

void mpuselect(int i) { // sets AD0 pins LOW or HIGH to select desired MPU
  switch (i) {
    case 0:
      digitalWrite(AD0_0, LOW);
      digitalWrite(AD0_1, HIGH);
      digitalWrite(AD0_2, HIGH);
      break;
    case 1:
      digitalWrite(AD0_0, HIGH);
      digitalWrite(AD0_1, LOW);
      digitalWrite(AD0_2, HIGH);
      break;
    case 2:
      digitalWrite(AD0_0, HIGH);
      digitalWrite(AD0_1, HIGH);
      digitalWrite(AD0_2, LOW);
      break;
    default:
      digitalWrite(AD0_0, LOW);
      digitalWrite(AD0_1, HIGH);
      digitalWrite(AD0_2, HIGH);
      break;

  }
}
