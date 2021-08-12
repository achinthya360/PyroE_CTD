#include "MPU9250.h"

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire, 0x68);
int status;

#define TCAADDR 0x70

void setup() {
  // serial to display data
  Serial.begin(115200);
  while (!Serial) {}

  for (uint8_t t = 0; t < 3; t++) {
    tcaselect(t);
    // start communication with IMU
    status = -1;
    while(status < 0){
      status = IMU.begin();
      Serial.println(status);
    }
    if (status < 0) {
      Serial.println("IMU initialization unsuccessful");
      Serial.println("Check IMU wiring or try cycling power");
      Serial.print("Status: ");
      Serial.println(status);
      //    while(1) {}
      continue;
    }
    else{
      Serial.println(status);
    }
    // setting the accelerometer full scale range to +/-8G
    IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
    // setting the gyroscope full scale range to +/-500 deg/s
    IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
    // setting DLPF bandwidth to 20 Hz
    IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
    // setting SRD to 19 for a 50 Hz update rate
    IMU.setSrd(19);
    calibrateMPU();
  }
}

void loop() {
  for (uint8_t t = 0; t < 3; t++) {
    tcaselect(t);
    // read the sensor
    IMU.readSensor();

    // display the data
    Serial.print(t);
    Serial.print("\t");
    Serial.print(IMU.getAccelX_mss(), 6);
    Serial.print("\t");
    Serial.print(IMU.getAccelY_mss(), 6);
    Serial.print("\t");
    Serial.print(IMU.getAccelZ_mss(), 6);
    Serial.print("\t");
    Serial.print(IMU.getGyroX_rads(), 6);
    Serial.print("\t");
    Serial.print(IMU.getGyroY_rads(), 6);
    Serial.print("\t");
    Serial.print(IMU.getGyroZ_rads(), 6);
    Serial.print("\t");
    Serial.println(millis());
    //  Serial.print(IMU.getMagX_uT(),6);
    //  Serial.print("\t");
    //  Serial.print(IMU.getMagY_uT(),6);
    //  Serial.print("\t");
    //  Serial.print(IMU.getMagZ_uT(),6);
    //  Serial.print("\t");
    //  Serial.println(IMU.getTemperature_C(),6);
    delay(20);
  }
}

void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void calibrateMPU() {
  status = IMU.calibrateAccel(); // (optional) int calibrateAccel() This function will estimate the bias and scale factor needed to calibrate the accelerometers. This function works one axis at a time and needs to be run for all 6 sensor orientations. After it has collected enough sensor data, it will estimate the bias and scale factor for all three accelerometer channels and apply these corrections to the measured data. Accelerometer calibration only needs to be performed once on the IMU, the get and set functions detailed below can be used to retrieve the estimated bias and scale factors and use them during future power cycles or operations with the IMU. This function returns a positive value on success and a negative value on failure.
  Serial.print("calibration status: ");
  Serial.println(status);

  // (optional) float getAccelBiasX_mss() This function returns the current accelerometer bias in the X direction in units of m/s/s.

  float axb;
  axb = IMU.getAccelBiasX_mss();
  Serial.println(axb);

  //(optional) float getAccelScaleFactorX() This function returns the current accelerometer scale factor in the X direction

  float axs;
  axs = IMU.getAccelScaleFactorX();

  // (optional) float getAccelBiasY_mss() This function returns the current accelerometer bias in the Y direction in units of m/s/s.

  float ayb;
  ayb = IMU.getAccelBiasY_mss();

  // (optional) float getAccelScaleFactorY() This function returns the current accelerometer scale factor in the Y direction.

  float ays;
  ays = IMU.getAccelScaleFactorY();

  // (optional) float getAccelBiasZ_mss() This function returns the current accelerometer bias in the Z direction in units of m/s/s.

  float azb;
  azb = IMU.getAccelBiasZ_mss();

  //(optional) float getAccelScaleFactorZ() This function returns the current accelerometer scale factor in the Z direction.

  float azs;
  azs = IMU.getAccelScaleFactorZ();

  IMU.readSensor();

  //// (optional) void setAccelCalX(float bias,float scaleFactor) This function sets the accelerometer bias (m/s/s) and scale factor being used in the X direction to the input values.
  //float axb = 0.01; // accel bias of 0.01 m/s/s
  //float axs = 0.97; // accel scale factor of 0.97
//  IMU.setAccelCalX(axb,axs);
  IMU.setAccelCalX(IMU.getAccelX_mss(),axs);
  //
  //// (optional) void setAccelCalY(float bias,float scaleFactor) This function sets the accelerometer bias (m/s/s) and scale factor being used in the Y direction to the input values.
  //float ayb = 0.01; // accel bias of 0.01 m/s/s
  //float ays = 0.97; // accel scale factor of 0.97
//  IMU.setAccelCalY(ayb,ays);
  IMU.setAccelCalY(IMU.getAccelY_mss(),ays);
  //
  //// (optional) void setAccelCalZ(float bias,float scaleFactor) This function sets the accelerometer bias (m/s/s) and scale factor being used in the Z direction to the input values.
  //float azb = 0.01; // accel bias of 0.01 m/s/s
  //float azs = 0.97; // accel scale factor of 0.97
//  IMU.setAccelCalZ(azb,azs);
  IMU.setAccelCalZ(IMU.getAccelZ_mss(),azs);
}
