#include "MPU9250.h"

#define TCAADDR 0x70

MPU9250 mpu, mpu2, mpu3;

void setup() {
  Serial.begin(115200);
  Wire.begin();
//  Wire.setClock(10);
  delay(2000);

  Serial.println("Beginning Calibration");

  tcaselect(0);

  if (!mpu.setup(0x68)) {  // change to your own address
    while (1) {
      Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
      delay(5000);
    }
  }

  mpu.calibrateAccelGyro();

  tcaselect(1);
  if (!mpu2.setup(0x68)) {  // change to your own address
    while (1) {
      Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
      delay(5000);
    }
  }

  mpu2.calibrateAccelGyro();

  tcaselect(2);
  if (!mpu3.setup(0x68)) {  // change to your own address
    while (1) {
      Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
      delay(5000);
    }
  }

  mpu3.calibrateAccelGyro();

  Serial.println("Beginning Data Collection");
}

void loop() {
  tcaselect(0);
  if (mpu.update()) {
    static uint32_t prev_ms = millis();
    if (millis() > prev_ms + 100) {
      print_roll_pitch_yaw();
      prev_ms = millis();
    }
  }

  tcaselect(1);
  if (mpu2.update()) {
    static uint32_t prev_ms2 = millis();
    if (millis() > prev_ms2 + 100) {
      print2_roll_pitch_yaw();
      prev_ms2 = millis();
    }
  }

  tcaselect(2);
  if (mpu3.update()) {
    static uint32_t prev_ms2 = millis();
    if (millis() > prev_ms2 + 100) {
      print3_roll_pitch_yaw();
      prev_ms2 = millis();
    }
  }
}

void print_roll_pitch_yaw() {
  Serial.print(millis());
  Serial.print(", 1, ");
  //  Serial.print("1: Yaw , Pitch , Roll : ");
  Serial.print(mpu.getYaw(), 2);
  Serial.print(", ");
  Serial.print(mpu.getPitch(), 2);
  Serial.print(", ");
  Serial.print(mpu.getRoll(), 2);

  Serial.print(", ");
  //  Serial.print(",\tAccX , AccY , AccZ : ");
  Serial.print(mpu.getAccX(), 2);
  Serial.print(", ");
  Serial.print(mpu.getAccY(), 2);
  Serial.print(", ");
  Serial.print(mpu.getAccZ(), 2);
}

void print2_roll_pitch_yaw() {
  Serial.print(", ");
  Serial.print(millis());
  Serial.print(", 2, ");
  //  Serial.print("2: Yaw2, Pitch2, Roll2: ");
  Serial.print(mpu2.getYaw(), 2);
  Serial.print(", ");
  Serial.print(mpu2.getPitch(), 2);
  Serial.print(", ");
  Serial.print(mpu2.getRoll(), 2);

  Serial.print(", ");
  //  Serial.print(",\tAccX2 , AccY2 , AccZ2 : ");
  Serial.print(mpu2.getAccX(), 2);
  Serial.print(", ");
  Serial.print(mpu2.getAccY(), 2);
  Serial.print(", ");
  Serial.print(mpu2.getAccZ(), 2);
}

void print3_roll_pitch_yaw() {
  Serial.print(", ");
  Serial.print(millis());
  Serial.print(", 3, ");
  //  Serial.print("3: Yaw3, Pitch3, Roll3: ");
  Serial.print(mpu3.getYaw(), 2);
  Serial.print(", ");
  Serial.print(mpu3.getPitch(), 2);
  Serial.print(", ");
  Serial.print(mpu3.getRoll(), 2);

  Serial.print(", ");
  //  Serial.print(",\tAccX3 , AccY3 , AccZ3 : ");
  Serial.print(mpu3.getAccX(), 2);
  Serial.print(", ");
  Serial.print(mpu3.getAccY(), 2);
  Serial.print(", ");
  Serial.println(mpu3.getAccZ(), 2);
}

void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}
