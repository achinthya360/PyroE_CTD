// Date and time functions using a DS3231 RTC connected via I2C and Wire lib
// RTC comes from Featherwing attachment
#include <Wire.h>
#include "RTClib.h"

// library for sleep mode
#include <ArduinoLowPower.h>

// library setup for EC circuit UART
#include <SoftwareSerial.h>

// libary for the pressure sensor
#include <MS5803_14.h>

// libraries for temperature sensors
#include <OneWire.h>
#include <DallasTemperature.h>

// define interrupt pin to use for wakeup
const int int_pin = 5;

RTC_DS3231 rtc; //define real-time clock

OneWire oneWire(6); // Define the OneWire port for temperature.
DallasTemperature sensors(&oneWire); //Define DallasTemperature input based on OneWire.

SoftwareSerial ecSerial(12, 13); // Define the SoftwareSerial port for conductivity.

const int chipSelect = 4; //sets chip select pin for SD card reader
char datalogFileName[12];

/* This integer specifies how high accuracy you want your pressure sensor to be (oversampling resolution).
   Ok values: 256, 512, 1024, 2048, or 4096 (Higher = more accuracy but slower sampling frequency**)
 * ** There is no reason not to use the highest accuracy. This is because the datalogging rate is set by the
   sampling/response frequency of the ec sensor [default = 1 sec] (this is to avoid the case where both sensors send data at the same time). */

#define PRESSURE_SENSOR_RESOLUTION 4096
MS_5803 sensor = MS_5803(PRESSURE_SENSOR_RESOLUTION); // Define pressure sensor.
double pressure_abs; //define absolute pressure variable

//Declare global variables for electrical conductivity
float EC_float = 0;
char EC_data[48]; // A 48 byte character array to hold incoming data from the conductivity circuit.
char *EC; // Character pointer for string parsing.
byte received_from_sensor = 0; // How many characters have been received.
byte string_received = 0; // Whether it received a string from the EC circuit.

#define EC_SAMPLING_FREQUENCY 1 // Set the requested sampling frequency of the conductivity probe in seconds (NO Decimals) (this by extension sets the overall frequency of logging).

/* This integer specifies how high resolution you want your temperature sensors to be.
   Ok values: 9,10,11,12 (Higher = more accuracy but slower sampling frequency**)
 * ** There is no reason not to use the highest accuracy. This is because the datalogging rate is set by the
   sampling/response frequency of the ec sensor [default = 1 second] (this is to avoid the case where both sensors send data at the same time). */
#define TEMP_SENSOR_RESOLUTION 12

//Declare global temperature variables.
float tempA;
float tempB;
float tempC;
float tempD;
float tempE;
int tempADelayStartTime; // Define a variable to mark when we requested a temperature measurement from A so we can wait the required delay before reading the value.
int tempBDelayStartTime; // Define a variable to mark when we requested a temperature measurement from B so we can wait the required delay before reading the value.
int tempCDelayStartTime; // Define a variable to mark when we requested a temperature measurement from C so we can wait the required delay before reading the value.
int tempDDelayStartTime; // Define a variable to mark when we requested a temperature measurement from D so we can wait the required delay before reading the value.
int tempEDelayStartTime; // Define a variable to mark when we requested a temperature measurement from E so we can wait the required delay before reading the value.
int requiredmeasurementDelay = sensors.millisToWaitForConversion(TEMP_SENSOR_RESOLUTION);

// use a variable to store the number of measurements taken by sensors on each wakeup
int measurements = 0;


void setup()  
{
  //Initialize real-time clock
  if (rtc.lostPower()) {

    //reset RTC with time when code was compiled if RTC loses power
    Serial.println("RTC lost power, lets set the time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  }

  delay(250);   // Wait a quarter second to continue.

   //Initialize pressure sensor
  Serial.println("-- Pressure Sensor Info: --");
  sensor.initializeMS_5803(); // Initialize pressure sensor
  Serial.println("---------------------------");

  // Initialize conductivity sensor and circuit
  ecSerial.begin(9600); // Set baud rate for conductivity circuit.

  do {

    ecSerial.write('i');  // Tell electrical conductivity board to reply with the board information by sending the 'i' character ...
    ecSerial.write('\r'); // ... Finish the command with the charage return character.
    received_from_sensor = ecSerial.readBytesUntil('\r', EC_data, 30); // Wait for the ec circut to send the data before moving on...
    EC_data[received_from_sensor] = 0; // Null terminate the data by setting the value after the final character to 0.

  } while (EC_data[1] != 'I'); // Keep looping until the ecSerial has sent the board info string (also indicating it has booted up, I think...)

  Serial.print("EC Board Info (Format: ?I,[board type],[Firmware Version]) -> "); Serial.println(EC_data);

  delay(10);
  ecSerial.write('C');  // Tell electrical conductivity board to continously ("C") transmit measurements ...
  ecSerial.write(',');  //
  ecSerial.print(EC_SAMPLING_FREQUENCY);    // ... every x seconds (here x is the EC_SAMPLING_FREQUENCY variable)
  ecSerial.write('\r'); // Finish the command with the carrage return character.

  received_from_sensor = ecSerial.readBytesUntil('\r', EC_data, 10); // keep reading the reply until the return character is recived (or it gets to be 10 characters long, which shouldn't happen)
  EC_data[received_from_sensor] = 0; // Null terminate the data by setting the value after the final character to 0.
  Serial.print("EC Frequency Set Sucessfully? -> "); Serial.println(EC_data);
  Serial.println("--- Starting Datalogging ---");

  // Intialize temperature sensors
  sensors.begin();  
  sensors.setResolution(TEMP_SENSOR_RESOLUTION);  // Set the resolution (accuracy) of the temperature sensors.
  sensors.requestTemperatures(); // on the first pass request all temperatures in a blocking way to start the variables with true data.
  tempA = get_temp_c_by_index(0);
  tempB = get_temp_c_by_index(1);
  tempC = get_temp_c_by_index(2);
  tempD = get_temp_c_by_index(3);
  tempE = get_temp_c_by_index(4);

  sensors.setWaitForConversion(false);  // Now tell the Dallas Temperature library to not block this script while it's waiting for the temperature measurement to happen
  
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);

  // setup interrupt pin
  pinMode(int_pin, INPUT_PULLUP);
  // set interrupt pin as wakeup pin
  LowPower.attachInterruptWakeup(int_pin, turn_on, CHANGE); 

  // delay to wait for all set up to complete 
  delay(1000);
  Serial.print("Interrupt set on pin: ");
  Serial.println(int_pin);
} 

void loop()  
{ 
  // turn LED on to show Feather is awake
  digitalWrite(LED_BUILTIN, HIGH);
  
  //Read electrical conductivity sensor
  if (ecSerial.available() > 0) {

    received_from_sensor = ecSerial.readBytesUntil(13, EC_data, 48);
    EC_data[received_from_sensor] = 0; // Null terminate the data by setting the value after the final character to 0.

    if ((EC_data[0] >= 48) && (EC_data[0] <= 57)) { // Parse data, if EC_data begins with a digit, not a letter (testing ASCII values).

      parse_data();

    }

   // Read the temperature sensors.
    if (millis() - tempADelayStartTime > requiredmeasurementDelay) { // wait for conversion to happen before attempting to read temp probe A's value;
      tempA = get_temp_c_by_index(0);
      sensors.requestTemperaturesByIndex(0);  // request temp sensor A start mesuring so it can be read on the following loop (if enough time elapses).
      tempADelayStartTime = millis();  // mark when we made the request to make sure we wait long enough before reading it.
    }
    if (millis() - tempBDelayStartTime > requiredmeasurementDelay) { // wait for conversion to happen before attempting to read temp probe B's value;
      tempB = get_temp_c_by_index(1);
      sensors.requestTemperaturesByIndex(1);  // request temp sensor B start mesuring so it can be read on the following loop (if enough time elapses).
      tempBDelayStartTime = millis();  // mark when we made the request to make sure we wait long enough before reading it.
    }
    if (millis() - tempCDelayStartTime > requiredmeasurementDelay) { // wait for conversion to happen before attempting to read temp probe B's value;
      tempC = get_temp_c_by_index(2);
      sensors.requestTemperaturesByIndex(2);  // request temp sensor C start mesuring so it can be read on the following loop (if enough time elapses).
      tempCDelayStartTime = millis();  // mark when we made the request to make sure we wait long enough before reading it.
    }
    if (millis() - tempDDelayStartTime > requiredmeasurementDelay) { // wait for conversion to happen before attempting to read temp probe B's value;
      tempD = get_temp_c_by_index(3);
      sensors.requestTemperaturesByIndex(3);  // request temp sensor D start mesuring so it can be read on the following loop (if enough time elapses).
      tempDDelayStartTime = millis();  // mark when we made the request to make sure we wait long enough before reading it.
    }
    if (millis() - tempEDelayStartTime > requiredmeasurementDelay) { // wait for conversion to happen before attempting to read temp probe B's value;
      tempE = get_temp_c_by_index(4);
      sensors.requestTemperaturesByIndex(4);  // request temp sensor E start mesuring so it can be read on the following loop (if enough time elapses).
      tempEDelayStartTime = millis();  // mark when we made the request to make sure we wait long enough before reading it.
    }

    sensor.readSensor(); //read pressure sensor
    pressure_abs = sensor.pressure();

    DateTime now = rtc.now(); //check RTC
    char dateTimeString[40];
    get_date_time_string(dateTimeString, now);

    //output readings to serial
    Serial.print(dateTimeString);
    Serial.print(",");
    Serial.print(pressure_abs);
    Serial.print(",");
    Serial.print(tempA);
    Serial.print(",");
    Serial.print(tempB);
    Serial.print(",");
    Serial.print(tempC);
    Serial.print(",");
    Serial.print(tempD);
    Serial.print(",");
    Serial.print(tempE);
    Serial.print(",");
    Serial.println(EC);
    Serial.print(", ");
    Serial.println(millis());
  }

  // Tip: For a slower overall logging frequency, set the EC_SAMPLING_FREQUENCY variable rather than adding a delay (this will avoid the possibility of garbled ec sensor readings)
   
  delay(100);
  measurements++;
  if(measurements > 100){
    // Feather falls asleep after 100 sensor measurements
    Serial.println("Sleeping now");
    digitalWrite(LED_BUILTIN, LOW);
    LowPower.sleep();
  }
}

float get_temp_c_by_index(int sensor_index) {
  
  float value = sensors.getTempCByIndex(sensor_index);
  if (value == DEVICE_DISCONNECTED_C) {
    return NAN; // Return Not a Number (NAN) to indicate temperature probe has error or is disconnected.
  } else {
    return value; // otherwise return the measured value.
  }
}

// function run on every interrupt wakeup
// TODO: Serial does not turn on when MCU turns back on
void turn_on(){
  measurements = 0;
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("Turned back on");
}

void parse_data() { // Parses data from the EC Circuit.

  EC = strtok(EC_data, ",");

}

void get_date_time_string(char* outStr, DateTime date) {
  // outputs the date as a date time string,
  sprintf(outStr, "%02d/%02d/%02d,%02d:%02d:%02d", date.month(), date.day(), date.year(), date.hour(), date.minute(), date.second());
  // Note: If you would like the date & time to be seperate columns chabge the space in the formatting string to a comma - this works because the file type is CSV (Comma Seperated Values)
}
