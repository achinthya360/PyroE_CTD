// REQUIRED LIBRARIES
#include <Servo.h>
#include <AccelStepper.h>
#include <TimeLib.h>
#include <SPI.h> //serial peripheral interface for SD card reader
#include <SD.h> //library for SD card reader
#include "HX711.h" // strain gauges
#include "ICM_20948.h" // Tail MPU

// CONSTANTS AND VARIABLE DECLARATIONS //

// SD CARD
char datalogFileName[12] = "datalog.csv";
int chipSelect = BUILTIN_SDCARD;

// MCU COMMUNICATION
const int trig = 2;
bool wake = false;

// HEADING MPU
int baudrate = 115200;
int databytes;

// TAIL MPU
#define SERIAL_PORT Serial
#define WIRE_PORT Wire
#define AD0_VAL 1
ICM_20948_I2C myICM;
//Global variables to zero out gravity vector
double quatx, quaty,quatz;
double gravityX = 0;
double gravityY = 0;
double gravityZ = 0;
double newx = 0;
double newy = 0;
double newz = 0;
String mpuOut = String(newx,4)+","+String(newy,4)+","+String(newz,4);

// LEAK SENSOR
int leakPin = 23;
bool leak = false;

// WATER LEVEL SENSOR
int wetPin = 22;
bool wet = false;

// SHAFT ANGLE SENSOR
int shaftAnglePin = 21;
bool shaftAngle;

// STRAIN GAUGES
const int LOADCELL1_DOUT_PIN = 17;
const int LOADCELL1_SCK_PIN = 16;
const int LOADCELL2_DOUT_PIN = 25;
const int LOADCELL2_SCK_PIN = 24;
HX711 scale1;
HX711 scale2;
float scale1reading;
float scale2reading;

// MOTORS
int bs1ONpin = 5;
int bs2ONpin = 4;
int bs1controlPin = A0;
int bs2controlPin = A1;
Servo ballast1;
Servo ballast2;

// 48V TAIL MOTOR CONTROL
// powering on
int magSwitchTimeOn;
int magSwitchPin = 12;    // TODO: CHANGE THIS
int tailMotorOnPin = 3;   // TODO: CHANGE THIS

// turning control
#define kinematic_coeff 3.35
const float max_speed = 5000; // maximum possible ~ 5000

float microsteps = 800; // from driver      CHANGE THIS BASED ON DRIVER STEPS/FULL CIRCLE
float theta_max = 30; // degree amplitude   CHANGE THIS FOR DESIRED AMPLITUDE
long amplitude = microsteps / 360 * theta_max; // microsteps needed to reach wanted angle
//float accel = 90000;  // maximum possible ~ 85000
float freq_wanted = 1; // CHANGE THIS FOR DESIRED FREQUENCY

// theoretical calculations assume constant linear stepping acceleration
//float freq = 3.35 * sqrt(accel/(theta_max * microsteps)); // could be used to back calculate accel
float a = pow(freq_wanted*sqrt(theta_max*microsteps) / kinematic_coeff, 2); // a = (f * sqrt(theta * microsteps) / 3.35) ^ 2

// Define a stepper and the pins it will use
AccelStepper stepper(AccelStepper::DRIVER, 3, 2); // Using stepper pins (3 for pulse, 2 for dir) TODO: SET THESE PINS CORRECTLY


void setup()
{
  // only for testing with computer
  Serial.begin(9600);

  // RTC
  // set the Time library to use Teensy 3.0's RTC to keep time
  setSyncProvider(getTeensy3Time);

  // SD CARD
  while (!SD.begin(chipSelect)) {
      Serial.println("Card failed, or not present");
      delay(1000);
  }

  // MCU COMMUNICATION

  //  softSerial.begin(9600);
  //  Serial1.begin(9600);
  //  pinMode(LED_BUILTIN, OUTPUT);
  //  pinMode(trig, OUTPUT);
  //  digitalWrite(trig, HIGH);
  //  digitalWrite(LED_BUILTIN, HIGH);

  // Heading MPU
  Serial1.begin(115200);

  // SENSOR INTEGRATION

  pinMode(leakPin, INPUT); // leak sensor
  pinMode(wetPin, INPUT);  // water level sensor
  pinMode(shaftAnglePin, INPUT);  // shaft angle sensor
  // strain gauges
  scale1.begin(LOADCELL1_DOUT_PIN, LOADCELL1_SCK_PIN);
  scale2.begin(LOADCELL2_DOUT_PIN, LOADCELL2_SCK_PIN);
  scale1.set_scale();        // calibrate strain gauges here
  scale1.tare();
  scale2.set_scale();        // calibrate strain gauges here
  scale2.tare();
  // Tail MPU
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
  bool initialized = false;
  while (!initialized)
  {
    myICM.begin(WIRE_PORT, AD0_VAL);
    if (myICM.status == ICM_20948_Stat_Ok)
    {
      initialized = true;
    }
  }
  SERIAL_PORT.println(F("Tail MPU connected!"));
  bool success = true; // Use success to show if the DMP configuration was successful
  // Initialize the DMP. initializeDMP is a weak function. You can overwrite it if you want to e.g. to change the sample rate
  success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);
  Serial.println("initializedDMP");
  // Enable the DMP Game Rotation Vector sensor
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
  Serial.println("enabled Game Rotation Vector");
  //added linear acceleration RC10082021
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_LINEAR_ACCELERATION) == ICM_20948_Stat_Ok);
  Serial.println("enabled linear acceleration");
  // Enable the FIFO
  success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);
  Serial.println("enabled FIFO");
  // Enable the DMP
  success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);
  Serial.println("enabledDMP");
  // Reset DMP
  success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);
  Serial.println("reset DMP");
  // Reset FIFO
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);
  Serial.println("reset FIFO");
  // Check success
  if (success)
  {
    SERIAL_PORT.println(F("DMP enabled!"));
  }
  else
  {
    SERIAL_PORT.println(F("Enable DMP failed!"));
    SERIAL_PORT.println(F("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
  }
  orientIMUSensor();//8-10 sec calibration to get gravity vector

  
  // MOTOR CONTROL   TODO: UNCOMMENT FOLLOWING LOWERCASE LINES
//  pinMode(bs1ONpin, OUTPUT);
//  digitalWrite(bs2ONpin, LOW);
//  pinMode(bs2ONpin, OUTPUT);
//  digitalWrite(bs2ONpin, LOW);
//  ballast1.attach(bs1controlPin);
//  ballast2.attach(bs2controlPin);
  // 48V TAIL MOTOR CONTROL
  // powering on with magnetic switch
  pinMode(magSwitchPin, INPUT);
  pinMode(tailMotorOnPin, OUTPUT);
  digitalWrite(tailMotorOnPin, LOW);
  // turning control
  stepper.setMaxSpeed(max_speed);
  stepper.setAcceleration(a);
  // this function should center the tail to 0 degrees (may need slight degree offset)
//  centerTailMotor();  TODO: UNCOMMENT THIS TO USE SHAFT ANGLE TO CENTER TAIL

  // initiate first sweep of motor
  stepper.moveTo(amplitude);
}

void loop()
{
  // MCU COMMUNICATION (to/from CTD)

  //  if(!wake and Serial1.read() == 'o'){
  //    pinMode(trig, OUTPUT); // connect and pull Feather's ENable pin to GND
  //    digitalWrite(trig, LOW);
  //    digitalWrite(LED_BUILTIN, LOW);
  //    delay(20000);
  //    wake = true;
  //  }
  //  if(wake){
  //    pinMode(trig, INPUT); // disconnect and let Feather's internal pullup turn on ENable pin
  ////    digitalWrite(trig, HIGH);
  //    digitalWrite(LED_BUILTIN, HIGH);
  //    wake = false;
  //  }

  // Heading MPU
  // TODO: ADD RUSSELL'S MODULE READING CODE


  // SENSOR INTEGRATION

  // monitor leaking based on 0 or 1 from leak sensor
  leak = (digitalRead(leakPin) == 1) ? true : false;

  // monitor if wet or not using water level sensor
  wet = (digitalRead(wetPin) == 1) ? true : false;

  // monitor if tail is centered
  shaftAngle = (digitalRead(shaftAnglePin) == 1) ? true : false;

  // monitor strain gauges for velocity x and y
  scale1reading = scale1.get_units(); // average 10 readings for scale 1
  scale2reading = scale2.get_units(); // average 10 readings for scale 2
  
  // tail MPU
  readTailMPU();

  // MOTOR CONTROL
  //  ballast1.write(//insert wanted position here);
  //  ballast2.write(//insert wanted position here);
  // 48V TAIL MOTOR CONTROL
  // powering on with magnetic switch
  
  // If at the end of travel go to the other end
  if (stepper.distanceToGo() == 0)
    stepper.moveTo(-stepper.currentPosition());
  stepper.run();



    // TESTING WITH COMPUTER
    Serial.print(month());
    Serial.print(" ");
    Serial.print(day());
    Serial.print(" ");
    Serial.print(year());
    Serial.print(" ");
    Serial.print(hour());
    printDigits(minute());
    printDigits(second());
    Serial.print(", Leak: ");
    Serial.print(leak);
    Serial.print(", Water Level: ");
    Serial.print(wet);
    Serial.print(", Shaft Angle:");
    Serial.print(shaftAngle);
    Serial.print(", Scale 1: ");
    Serial.print(scale1reading);
    Serial.print(", Scale 2: ");
    Serial.print(scale2reading);
    Serial.print(", MPU: ");
    Serial.println(mpuOut);

  // SD CARD WRITE
  File dataFile = SD.open(datalogFileName, FILE_WRITE);
  if (dataFile) {
    dataFile.print(month());
    dataFile.print(", ");
    dataFile.print(day());
    dataFile.print(", ");
    dataFile.print(year());
    dataFile.print(", ");
    dataFile.print(hour());
    SDprintDigits(minute(), dataFile);
    SDprintDigits(second(), dataFile);
    dataFile.print(", Leak:, ");
    dataFile.print(leak);
    dataFile.print(", Water Level:, ");
    dataFile.print(wet);
    dataFile.print(", Shaft Angle:, ");
    dataFile.print(shaftAngle);
    dataFile.print(", Scale 1:, ");
    dataFile.print(scale1reading);
    dataFile.print(", Scale 2:, ");
    dataFile.print(scale2reading);
    dataFile.print(", MPU:, ");
    dataFile.println(mpuOut);
    dataFile.close();
  }
}

void centerTailMotor(){
  stepper.moveTo(amplitude);
  // sweep the amplitude cone until the shaft angle sensor detects the tail centered under it 
  while(digitalRead(shaftAnglePin) != 1){
    stepper.run();
    if(stepper.distanceToGo() == 0){
      stepper.moveTo(-amplitude);
    }
  }
  stepper.stop();
  stepper.setCurrentPosition(0);
}

void tailMotorOn(){
  magSwitchTimeOn = 0; 
  Serial.println("Turned back on");
}

//RTC helper functions
time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

void printDigits(int digits){
  // utility function for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if(digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

void SDprintDigits(int digits, File dataFile){
  // utility function for digital clock display: prints preceding colon and leading 0
  dataFile.print(":");
  if(digits < 10)
    dataFile.print('0');
  dataFile.print(digits);
}

// MPU Helper Functions
// initializeDMP is a weak function. Let's overwrite it so we can increase the sample rate
//this piece of code comes from Sparkfun as they have NDA To program the DMP from Invensys
ICM_20948_Status_e ICM_20948::initializeDMP(void)
{
  // The ICM-20948 is awake and ready but hasn't been configured. Let's step through the configuration
  // sequence from InvenSense's _confidential_ Application Note "Programming Sequence for DMP Hardware Functions".

  ICM_20948_Status_e  result = ICM_20948_Stat_Ok; // Use result and worstResult to show if the configuration was successful
  ICM_20948_Status_e  worstResult = ICM_20948_Stat_Ok;

  // Normally, when the DMP is not enabled, startupMagnetometer (called by startupDefault, which is called by begin) configures the AK09916 magnetometer
  // to run at 100Hz by setting the CNTL2 register (0x31) to 0x08. Then the ICM20948's I2C_SLV0 is configured to read
  // nine bytes from the mag every sample, starting from the STATUS1 register (0x10). ST1 includes the DRDY (Data Ready) bit.
  // Next are the six magnetometer readings (little endian). After a dummy byte, the STATUS2 register (0x18) contains the HOFL (Overflow) bit.
  //
  // But looking very closely at the InvenSense example code, we can see in inv_icm20948_resume_akm (in Icm20948AuxCompassAkm.c) that,
  // when the DMP is running, the magnetometer is set to Single Measurement (SM) mode and that ten bytes are read, starting from the reserved
  // RSV2 register (0x03). The datasheet does not define what registers 0x04 to 0x0C contain. There is definitely some secret sauce in here...
  // The magnetometer data appears to be big endian (not little endian like the HX/Y/Z registers) and starts at register 0x04.
  // We had to examine the I2C traffic between the master and the AK09916 on the AUX_DA and AUX_CL pins to discover this...
  //
  // So, we need to set up I2C_SLV0 to do the ten byte reading. The parameters passed to i2cControllerConfigurePeripheral are:
  // 0: use I2C_SLV0
  // MAG_AK09916_I2C_ADDR: the I2C address of the AK09916 magnetometer (0x0C unshifted)
  // AK09916_REG_RSV2: we start reading here (0x03). Secret sauce...
  // 10: we read 10 bytes each cycle
  // true: set the I2C_SLV0_RNW ReadNotWrite bit so we read the 10 bytes (not write them)
  // true: set the I2C_SLV0_CTRL I2C_SLV0_EN bit to enable reading from the peripheral at the sample rate
  // false: clear the I2C_SLV0_CTRL I2C_SLV0_REG_DIS (we want to write the register value)
  // true: set the I2C_SLV0_CTRL I2C_SLV0_GRP bit to show the register pairing starts at byte 1+2 (copied from inv_icm20948_resume_akm)
  // true: set the I2C_SLV0_CTRL I2C_SLV0_BYTE_SW to byte-swap the data from the mag (copied from inv_icm20948_resume_akm)
  result = i2cControllerConfigurePeripheral(0, MAG_AK09916_I2C_ADDR, AK09916_REG_RSV2, 10, true, true, false, true, true); if (result > worstResult) worstResult = result;
  //
  // We also need to set up I2C_SLV1 to do the Single Measurement triggering:
  // 1: use I2C_SLV1
  // MAG_AK09916_I2C_ADDR: the I2C address of the AK09916 magnetometer (0x0C unshifted)
  // AK09916_REG_CNTL2: we start writing here (0x31)
  // 1: not sure why, but the write does not happen if this is set to zero
  // false: clear the I2C_SLV0_RNW ReadNotWrite bit so we write the dataOut byte
  // true: set the I2C_SLV0_CTRL I2C_SLV0_EN bit. Not sure why, but the write does not happen if this is clear
  // false: clear the I2C_SLV0_CTRL I2C_SLV0_REG_DIS (we want to write the register value)
  // false: clear the I2C_SLV0_CTRL I2C_SLV0_GRP bit
  // false: clear the I2C_SLV0_CTRL I2C_SLV0_BYTE_SW bit
  // AK09916_mode_single: tell I2C_SLV1 to write the Single Measurement command each sample
  result = i2cControllerConfigurePeripheral(1, MAG_AK09916_I2C_ADDR, AK09916_REG_CNTL2, 1, false, true, false, false, false, AK09916_mode_single); if (result > worstResult) worstResult = result;

  // Set the I2C Master ODR configuration
  // It is not clear why we need to do this... But it appears to be essential! From the datasheet:
  // "I2C_MST_ODR_CONFIG[3:0]: ODR configuration for external sensor when gyroscope and accelerometer are disabled.
  //  ODR is computed as follows: 1.1 kHz/(2^((odr_config[3:0])) )
  //  When gyroscope is enabled, all sensors (including I2C_MASTER) use the gyroscope ODR.
  //  If gyroscope is disabled, then all sensors (including I2C_MASTER) use the accelerometer ODR."
  // Since both gyro and accel are running, setting this register should have no effect. But it does. Maybe because the Gyro and Accel are placed in Low Power Mode (cycled)?
  // You can see by monitoring the Aux I2C pins that the next three lines reduce the bus traffic (magnetometer reads) from 1125Hz to the chosen rate: 68.75Hz in this case.
  result = setBank(3); if (result > worstResult) worstResult = result; // Select Bank 3
  uint8_t mstODRconfig = 0x04; // Set the ODR configuration to 1100/2^4 = 68.75Hz
  result = write(AGB3_REG_I2C_MST_ODR_CONFIG, &mstODRconfig, 1); if (result > worstResult) worstResult = result; // Write one byte to the I2C_MST_ODR_CONFIG register  

  // Configure clock source through PWR_MGMT_1
  // ICM_20948_Clock_Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
  result = setClockSource(ICM_20948_Clock_Auto); if (result > worstResult) worstResult = result; // This is shorthand: success will be set to false if setClockSource fails

  // Enable accel and gyro sensors through PWR_MGMT_2
  // Enable Accelerometer (all axes) and Gyroscope (all axes) by writing zero to PWR_MGMT_2
  result = setBank(0); if (result > worstResult) worstResult = result;                               // Select Bank 0
  uint8_t pwrMgmt2 = 0x40;                                                          // Set the reserved bit 6 (pressure sensor disable?)
  result = write(AGB0_REG_PWR_MGMT_2, &pwrMgmt2, 1); if (result > worstResult) worstResult = result; // Write one byte to the PWR_MGMT_2 register

  // Place _only_ I2C_Master in Low Power Mode (cycled) via LP_CONFIG
  // The InvenSense Nucleo example initially puts the accel and gyro into low power mode too, but then later updates LP_CONFIG so only the I2C_Master is in Low Power Mode
  result = setSampleMode(ICM_20948_Internal_Mst, ICM_20948_Sample_Mode_Cycled); if (result > worstResult) worstResult = result;

  // Disable the FIFO
  result = enableFIFO(false); if (result > worstResult) worstResult = result;

  // Disable the DMP
  result = enableDMP(false); if (result > worstResult) worstResult = result;

  // Set Gyro FSR (Full scale range) to 2000dps through GYRO_CONFIG_1
  // Set Accel FSR (Full scale range) to 4g through ACCEL_CONFIG
  ICM_20948_fss_t myFSS; // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors
  myFSS.a = gpm4;        // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
                         // gpm2
                         // gpm4
                         // gpm8
                         // gpm16
  myFSS.g = dps2000;     // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
                         // dps250
                         // dps500
                         // dps1000
                         // dps2000
  result = setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS); if (result > worstResult) worstResult = result;

  // The InvenSense Nucleo code also enables the gyro DLPF (but leaves GYRO_DLPFCFG set to zero = 196.6Hz (3dB))
  // We found this by going through the SPI data generated by ZaneL's Teensy-ICM-20948 library byte by byte...
  // The gyro DLPF is enabled by default (GYRO_CONFIG_1 = 0x01) so the following line should have no effect, but we'll include it anyway
  result = enableDLPF(ICM_20948_Internal_Gyr, true); if (result > worstResult) worstResult = result;

  // Enable interrupt for FIFO overflow from FIFOs through INT_ENABLE_2
  // If we see this interrupt, we'll need to reset the FIFO
  //result = intEnableOverflowFIFO( 0x1F ); if (result > worstResult) worstResult = result; // Enable the interrupt on all FIFOs

  // Turn off what goes into the FIFO through FIFO_EN_1, FIFO_EN_2
  // Stop the peripheral data from being written to the FIFO by writing zero to FIFO_EN_1
  result = setBank(0); if (result > worstResult) worstResult = result; // Select Bank 0
  uint8_t zero = 0;
  result = write(AGB0_REG_FIFO_EN_1, &zero, 1); if (result > worstResult) worstResult = result;
  // Stop the accelerometer, gyro and temperature data from being written to the FIFO by writing zero to FIFO_EN_2
  result = write(AGB0_REG_FIFO_EN_2, &zero, 1); if (result > worstResult) worstResult = result;

  // Turn off data ready interrupt through INT_ENABLE_1
  result = intEnableRawDataReady(false); if (result > worstResult) worstResult = result;

  // Reset FIFO through FIFO_RST
  result = resetFIFO(); if (result > worstResult) worstResult = result;

  // Set gyro sample rate divider with GYRO_SMPLRT_DIV
  // Set accel sample rate divider with ACCEL_SMPLRT_DIV_2
  ICM_20948_smplrt_t mySmplrt;
  //mySmplrt.g = 19; // ODR is computed as follows: 1.1 kHz/(1+GYRO_SMPLRT_DIV[7:0]). 19 = 55Hz. InvenSense Nucleo example uses 19 (0x13).
  //mySmplrt.a = 19; // ODR is computed as follows: 1.125 kHz/(1+ACCEL_SMPLRT_DIV[11:0]). 19 = 56.25Hz. InvenSense Nucleo example uses 19 (0x13).
  mySmplrt.g = 4; // 225Hz
  mySmplrt.a = 4; // 225Hz
  //mySmplrt.g = 8; // 112Hz
  //mySmplrt.a = 8; // 112Hz
  result = setSampleRate((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), mySmplrt); if (result > worstResult) worstResult = result;

  // Setup DMP start address through PRGM_STRT_ADDRH/PRGM_STRT_ADDRL
  result = setDMPstartAddress(); if (result > worstResult) worstResult = result; // Defaults to DMP_START_ADDRESS

  // Now load the DMP firmware
  result = loadDMPFirmware(); if (result > worstResult) worstResult = result;

  // Write the 2 byte Firmware Start Value to ICM PRGM_STRT_ADDRH/PRGM_STRT_ADDRL
  result = setDMPstartAddress(); if (result > worstResult) worstResult = result; // Defaults to DMP_START_ADDRESS

  // Set the Hardware Fix Disable register to 0x48
  result = setBank(0); if (result > worstResult) worstResult = result; // Select Bank 0
  uint8_t fix = 0x48;
  result = write(AGB0_REG_HW_FIX_DISABLE, &fix, 1); if (result > worstResult) worstResult = result;

  // Set the Single FIFO Priority Select register to 0xE4
  result = setBank(0); if (result > worstResult) worstResult = result; // Select Bank 0
  uint8_t fifoPrio = 0xE4;
  result = write(AGB0_REG_SINGLE_FIFO_PRIORITY_SEL, &fifoPrio, 1); if (result > worstResult) worstResult = result;

  // Configure Accel scaling to DMP
  // The DMP scales accel raw data internally to align 1g as 2^25
  // In order to align internal accel raw data 2^25 = 1g write 0x04000000 when FSR is 4g
  const unsigned char accScale[4] = {0x04, 0x00, 0x00, 0x00};
  result = writeDMPmems(ACC_SCALE, 4, &accScale[0]); if (result > worstResult) worstResult = result; // Write accScale to ACC_SCALE DMP register
  // In order to output hardware unit data as configured FSR write 0x00040000 when FSR is 4g
  const unsigned char accScale2[4] = {0x00, 0x04, 0x00, 0x00};
  result = writeDMPmems(ACC_SCALE2, 4, &accScale2[0]); if (result > worstResult) worstResult = result; // Write accScale2 to ACC_SCALE2 DMP register

  // Configure Compass mount matrix and scale to DMP
  // The mount matrix write to DMP register is used to align the compass axes with accel/gyro.
  // This mechanism is also used to convert hardware unit to uT. The value is expressed as 1uT = 2^30.
  // Each compass axis will be converted as below:
  // X = raw_x * CPASS_MTX_00 + raw_y * CPASS_MTX_01 + raw_z * CPASS_MTX_02
  // Y = raw_x * CPASS_MTX_10 + raw_y * CPASS_MTX_11 + raw_z * CPASS_MTX_12
  // Z = raw_x * CPASS_MTX_20 + raw_y * CPASS_MTX_21 + raw_z * CPASS_MTX_22
  // The AK09916 produces a 16-bit signed output in the range +/-32752 corresponding to +/-4912uT. 1uT = 6.66 ADU.
  // 2^30 / 6.66666 = 161061273 = 0x9999999
  const unsigned char mountMultiplierZero[4] = {0x00, 0x00, 0x00, 0x00};
  const unsigned char mountMultiplierPlus[4] = {0x09, 0x99, 0x99, 0x99};  // Value taken from InvenSense Nucleo example
  const unsigned char mountMultiplierMinus[4] = {0xF6, 0x66, 0x66, 0x67}; // Value taken from InvenSense Nucleo example
  result = writeDMPmems(CPASS_MTX_00, 4, &mountMultiplierPlus[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(CPASS_MTX_01, 4, &mountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(CPASS_MTX_02, 4, &mountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(CPASS_MTX_10, 4, &mountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(CPASS_MTX_11, 4, &mountMultiplierMinus[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(CPASS_MTX_12, 4, &mountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(CPASS_MTX_20, 4, &mountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(CPASS_MTX_21, 4, &mountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(CPASS_MTX_22, 4, &mountMultiplierMinus[0]); if (result > worstResult) worstResult = result;

  // Configure the B2S Mounting Matrix
  const unsigned char b2sMountMultiplierZero[4] = {0x00, 0x00, 0x00, 0x00};
  const unsigned char b2sMountMultiplierPlus[4] = {0x40, 0x00, 0x00, 0x00}; // Value taken from InvenSense Nucleo example
  result = writeDMPmems(B2S_MTX_00, 4, &b2sMountMultiplierPlus[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(B2S_MTX_01, 4, &b2sMountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(B2S_MTX_02, 4, &b2sMountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(B2S_MTX_10, 4, &b2sMountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(B2S_MTX_11, 4, &b2sMountMultiplierPlus[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(B2S_MTX_12, 4, &b2sMountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(B2S_MTX_20, 4, &b2sMountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(B2S_MTX_21, 4, &b2sMountMultiplierZero[0]); if (result > worstResult) worstResult = result;
  result = writeDMPmems(B2S_MTX_22, 4, &b2sMountMultiplierPlus[0]); if (result > worstResult) worstResult = result;

  // Configure the DMP Gyro Scaling Factor
  // @param[in] gyro_div Value written to GYRO_SMPLRT_DIV register, where
  //            0=1125Hz sample rate, 1=562.5Hz sample rate, ... 4=225Hz sample rate, ...
  //            10=102.2727Hz sample rate, ... etc.
  // @param[in] gyro_level 0=250 dps, 1=500 dps, 2=1000 dps, 3=2000 dps
  result = setGyroSF(4, 3); if (result > worstResult) worstResult = result; // 4 = 225Hz (see above), 3 = 2000dps (see above)

  // Configure the Gyro full scale
  // 2000dps : 2^28
  // 1000dps : 2^27
  //  500dps : 2^26
  //  250dps : 2^25
  const unsigned char gyroFullScale[4] = {0x10, 0x00, 0x00, 0x00}; // 2000dps : 2^28
  result = writeDMPmems(GYRO_FULLSCALE, 4, &gyroFullScale[0]); if (result > worstResult) worstResult = result;

  // Configure the Accel Only Gain: 15252014 (225Hz) 30504029 (112Hz) 61117001 (56Hz)
  //const unsigned char accelOnlyGain[4] = {0x03, 0xA4, 0x92, 0x49}; // 56Hz
  const unsigned char accelOnlyGain[4] = {0x00, 0xE8, 0xBA, 0x2E}; // 225Hz
  //const unsigned char accelOnlyGain[4] = {0x01, 0xD1, 0x74, 0x5D}; // 112Hz
  result = writeDMPmems(ACCEL_ONLY_GAIN, 4, &accelOnlyGain[0]); if (result > worstResult) worstResult = result;

  // Configure the Accel Alpha Var: 1026019965 (225Hz) 977872018 (112Hz) 882002213 (56Hz)
  //const unsigned char accelAlphaVar[4] = {0x34, 0x92, 0x49, 0x25}; // 56Hz
  const unsigned char accelAlphaVar[4] = {0x3D, 0x27, 0xD2, 0x7D}; // 225Hz
  //const unsigned char accelAlphaVar[4] = {0x3A, 0x49, 0x24, 0x92}; // 112Hz
  result = writeDMPmems(ACCEL_ALPHA_VAR, 4, &accelAlphaVar[0]); if (result > worstResult) worstResult = result;

  // Configure the Accel A Var: 47721859 (225Hz) 95869806 (112Hz) 191739611 (56Hz)
  //const unsigned char accelAVar[4] = {0x0B, 0x6D, 0xB6, 0xDB}; // 56Hz
  const unsigned char accelAVar[4] = {0x02, 0xD8, 0x2D, 0x83}; // 225Hz
  //const unsigned char accelAVar[4] = {0x05, 0xB6, 0xDB, 0x6E}; // 112Hz
  result = writeDMPmems(ACCEL_A_VAR, 4, &accelAVar[0]); if (result > worstResult) worstResult = result;

  // Configure the Accel Cal Rate
  const unsigned char accelCalRate[4] = {0x00, 0x00}; // Value taken from InvenSense Nucleo example
  result = writeDMPmems(ACCEL_CAL_RATE, 2, &accelCalRate[0]); if (result > worstResult) worstResult = result;

  // Configure the Compass Time Buffer. The I2C Master ODR Configuration (see above) sets the magnetometer read rate to 68.75Hz.
  // Let's set the Compass Time Buffer to 69 (Hz).
  const unsigned char compassRate[2] = {0x00, 0x45}; // 69Hz
  result = writeDMPmems(CPASS_TIME_BUFFER, 2, &compassRate[0]); if (result > worstResult) worstResult = result;

  // Enable DMP interrupt
  // This would be the most efficient way of getting the DMP data, instead of polling the FIFO
  //result = intEnableDMP(true); if (result > worstResult) worstResult = result;

  return worstResult;
}

void orientIMUSensor()
{
  Serial.println("Calibrating gravity vector - keep the sensor still or this might give inaccurate results");
  //Do exponential moving average over a certain time period and wait until vector settles at around 1g. Takes around 8 seconds.
  double totalAcc = sqrt(gravityX*gravityX + gravityY*gravityY + gravityZ*gravityZ);
  float ema = 0.01;
  
  while (totalAcc < 1000)
  {
    bool badReading = true; //Confirm if we have a valid result
    while (badReading)
    {
      int moreData = readIMUDMP();
      if (moreData >=0) //function returns -1 if no data was read.
      {
        gravityX = ema*quatx+(1-ema)*gravityX;
        gravityY = ema*quaty+(1-ema)*gravityY;
        gravityZ = ema*quatz+(1-ema)*gravityZ;
        badReading = false;
        totalAcc = sqrt(gravityX*gravityX + gravityY*gravityY + gravityZ*gravityZ);
        //Serial.println(String(gravityX)+","+String(gravityY)+","+String(gravityZ)+","+String(totalAcc));
      }  
      if(moreData ==0){
      delay(10);
      }
    }
  }
  Serial.println("Gravity vector initialised (will slowly update with changing conditions");
  //String outString = String(q0,4)+","+String(q1,4)+","+String(q2,4)+","+String(q3,4)+","+String(acc_x,0)+","+String(acc_y,0)+","+String(acc_z,0)+",";
}

int readIMUDMP()
{
    // Read any DMP data waiting in the FIFO
    // Note:
    //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFONoDataAvail if no data is available.
    //    If data is available, readDMPdataFromFIFO will attempt to read _one_ frame of DMP data.
    //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOIncompleteData if a frame was present but was incomplete
    //    readDMPdataFromFIFO will return ICM_20948_Stat_Ok if a valid frame was read.
    //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOMoreDataAvail if a valid frame was read _and_ the FIFO contains more (unread) data.

    icm_20948_DMP_data_t data;
    myICM.readDMPdataFromFIFO(&data);
    if (myICM.status == ICM_20948_Stat_FIFONoDataAvail)
    {
      return -1;
    }
  
    if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
    {
      //SERIAL_PORT.print(F("Received data! Header: 0x")); // Print the header in HEX so we can see what data is arriving in the FIFO
      //if ( data.header < 0x1000) SERIAL_PORT.print( "0" ); // Pad the zeros
      //if ( data.header < 0x100) SERIAL_PORT.print( "0" );
      //if ( data.header < 0x10) SERIAL_PORT.print( "0" );
      //SERIAL_PORT.println( data.header, HEX );


      //read Quarternion 
      if (((data.header & DMP_header_bitmap_Quat6) > 0)|| ((data.header & DMP_header_bitmap_Accel) > 0)) // Check for orientation data (Quat9)
      {
        // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
        // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
        // The quaternion data is scaled by 2^30.
  
        //Serial.printf("Quat6 data is: Q1:%ld Q2:%ld Q3:%ld\r\n", data.Quat6.Data.Q1, data.Quat6.Data.Q2, data.Quat6.Data.Q3);
  
  
        // Scale to +/- 1
        double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
        double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
        double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
  
        double q0 = sqrt( 1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
       
        //Serial.print(F("Q0:"));
        //Serial.print(q0, 3);
       // Serial.print(F("Q1:"));
        //Serial.print(q1, 3);
        //Serial.print(F(" Q2:"));
      //  Serial.print(q2, 3);
      //  Serial.print(F(" Q3:"));
       // Serial.println(q3, 3);
  
        //say that the quaternion represents the sensor orientation, but that the accelerometer also has a quaternion for the direction of force.
        // there exists a matrix operation such that gravity is always down.


        //get the accel data
        float acc_x = (float)data.Raw_Accel.Data.X; // Extract the raw accelerometer data
        float acc_y = (float)data.Raw_Accel.Data.Y;
        float acc_z = (float)data.Raw_Accel.Data.Z;
  
//        Serial.print(F("Accel: X:"));
//        Serial.print(acc_x,0);
//        Serial.print(F(" Y:"));
//        Serial.print(acc_y,0);
//        Serial.print(F(" Z:"));
//        Serial.println(acc_z,0);

        // Perform Quat6 rotation of raw acceleration values
      
        quatx = acc_x*(1-2*q2*q2-2*q3*q3)+acc_y*2*(q1*q2-q0*q3)+acc_z*2*(q1*q3+q0*q2);
        quaty = acc_x*2*(q1*q2+q0*q3)+acc_y*(1-2*q1*q1-2*q3*q3)+acc_z*2*(q2*q3-q0*q1);
        quatz = acc_x*2*(q1*q3-q0*q2)+acc_y*2*(q2*q3+q0*q1)+acc_z*(1-2*q1*q1-2*q2*q2);
        
      }
    }
    if (myICM.status != ICM_20948_Stat_FIFOMoreDataAvail) // If more data is available then we should read it right away - and not delay
    {
      return 0; // No more data
    }
    else
    {
      return 1; // More data
    }
}

void readTailMPU(){
  int moreData = readIMUDMP();//read quaternion and accel
   if (moreData >=0) //function returns -1 if no data was read.
   {

      double accelMag = sqrt(quatx*quatx+quaty*quaty+quatz*quatz);
      
      if ((accelMag <=1075)||(accelMag >=925))  //slowly update the gravity data whenever the sensor is still.
      {
        float ema = 0.01;
        gravityX = ema*quatx+(1-ema)*gravityX;
        gravityY = ema*quaty+(1-ema)*gravityY;
        gravityZ = ema*quatz+(1-ema)*gravityZ; 
      }

      // Re-orient the sensor so that gravity is down. See: https://stackoverflow.com/questions/1171849/finding-quaternion-representing-the-rotation-from-one-vector-to-another
      // Basically, we want to rotate the whole system so that the gravity vector (x,y,z) maps onto the unit vector (0,0,-1)
      // The cross-product of these two vectors is (-y,x,0) and the dot-product is (-z).
      // The rotation quaternion is therefore given by (q0,q1,q2,q3) = (sqrt(x*x+y*y+z*z)-z , -y , x , 0); although this needs normalising before application.
      double q0 = sqrt(gravityX*gravityX+gravityY*gravityY+gravityZ*gravityZ)-gravityZ;
      double q1 = -gravityY;
      double q2 = gravityX;
      double q3 = 0;
      double quatMag = sqrt(q0*q0+q1*q1+q2*q2+q3*q3);
      q0 /= quatMag;
      q1 /= quatMag;
      q2 /= quatMag;
      q3 /= quatMag;

      //Perform Quaternion rotation through matrix multiplication
      double newx = quatx*(1-2*q2*q2-2*q3*q3) + quaty*2*(q1*q2-q0*q3) + quatz*2*(q1*q3+q0*q2);
      double newy = quatx*2*(q1*q2+q0*q3) + quaty*(1-2*q1*q1-2*q3*q3) + quatz*2*(q2*q3-q0*q1);
      double newz = quatx*2*(q1*q3-q0*q2) + quaty*2*(q2*q3+q0*q1) + quatz*(1-2*q1*q1-2*q2*q2);     

      //..and subtract gravity from the Z axis:
      newz += sqrt(gravityX*gravityX+gravityY*gravityY+gravityZ*gravityZ);
     
      mpuOut = String(newx,4)+","+String(newy,4)+","+String(newz,4); // this should be the linear acceleration vector only if done right
   }
}
