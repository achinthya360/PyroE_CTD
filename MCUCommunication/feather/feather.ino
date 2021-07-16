//#include <SoftwareSerial.h>
#include <ArduinoLowPower.h>

// libraries for temperature sensor
#include <OneWire.h>
#include <DallasTemperature.h>

// define interrupt pin to use for wakeup
const int int_pin = 5;

//SoftwareSerial softSerial(10, 11);
char ip;

OneWire oneWire(6); // Define the OneWire port for temperature.
DallasTemperature sensors(&oneWire); //Define DallasTemperature input based on OneWire.

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
int tempADelayStartTime; // Define a variable to mark when we requested a temperature mesurement from A so we can wait the required delay before reading the value.
int tempBDelayStartTime; // Define a variable to mark when we requested a temperature mesurement from B so we can wait the required delay before reading the value.
int tempCDelayStartTime; // Define a variable to mark when we requested a temperature mesurement from C so we can wait the required delay before reading the value.
int tempDDelayStartTime; // Define a variable to mark when we requested a temperature mesurement from C so we can wait the required delay before reading the value.
int tempEDelayStartTime; // Define a variable to mark when we requested a temperature mesurement from C so we can wait the required delay before reading the value.
int requiredMesurementDelay = sensors.millisToWaitForConversion(TEMP_SENSOR_RESOLUTION);

// use a variable to store the number of measurements taken by sensors on each wakeup
int measurements = 0;


void setup()  
{
  sensors.begin();  // Intialize the temperature sensors.
  sensors.setResolution(TEMP_SENSOR_RESOLUTION);  // Set the resolution (accuracy) of the temperature sensors.
  sensors.requestTemperatures(); // on the first pass request all temperatures in a blocking way to start the variables with true data.
  tempA = get_temp_c_by_index(0);
  tempB = get_temp_c_by_index(1);
  tempC = get_temp_c_by_index(2);
  tempD = get_temp_c_by_index(3);
  tempE = get_temp_c_by_index(4);

  sensors.setWaitForConversion(false);  // Now tell the Dallas Temperature library to not block this script while it's waiting for the temperature mesurement to happen

  
//  softSerial.begin(9600);
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);

  // setup interrupt pin
  pinMode(int_pin, INPUT_PULLUP);
  // set interrupt pin as wakeup pin
  LowPower.attachInterruptWakeup(int_pin, turn_on, CHANGE); 
  delay(1000);
  Serial.print("Interrupt set on pin: ");
  Serial.println(int_pin);
} 

void loop()  
{ 
  digitalWrite(LED_BUILTIN, HIGH);
  // Read the temperature sensors.
    if (millis() - tempADelayStartTime > requiredMesurementDelay) { // wait for conversion to happen before attempting to read temp probe A's value;
      tempA = get_temp_c_by_index(0);
      sensors.requestTemperaturesByIndex(0);  // request temp sensor A start mesuring so it can be read on the following loop (if enough time elapses).
      tempADelayStartTime = millis();  // mark when we made the request to make sure we wait long enough before reading it.
    }
    if (millis() - tempBDelayStartTime > requiredMesurementDelay) { // wait for conversion to happen before attempting to read temp probe B's value;
      tempB = get_temp_c_by_index(1);
      sensors.requestTemperaturesByIndex(1);  // request temp sensor B start mesuring so it can be read on the following loop (if enough time elapses).
      tempBDelayStartTime = millis();  // mark when we made the request to make sure we wait long enough before reading it.
    }
    if (millis() - tempCDelayStartTime > requiredMesurementDelay) { // wait for conversion to happen before attempting to read temp probe B's value;
      tempC = get_temp_c_by_index(2);
      sensors.requestTemperaturesByIndex(2);  // request temp sensor C start mesuring so it can be read on the following loop (if enough time elapses).
      tempCDelayStartTime = millis();  // mark when we made the request to make sure we wait long enough before reading it.
    }
    if (millis() - tempDDelayStartTime > requiredMesurementDelay) { // wait for conversion to happen before attempting to read temp probe B's value;
      tempD = get_temp_c_by_index(3);
      sensors.requestTemperaturesByIndex(3);  // request temp sensor D start mesuring so it can be read on the following loop (if enough time elapses).
      tempDDelayStartTime = millis();  // mark when we made the request to make sure we wait long enough before reading it.
    }
    if (millis() - tempEDelayStartTime > requiredMesurementDelay) { // wait for conversion to happen before attempting to read temp probe B's value;
      tempE = get_temp_c_by_index(4);
      sensors.requestTemperaturesByIndex(4);  // request temp sensor E start mesuring so it can be read on the following loop (if enough time elapses).
      tempEDelayStartTime = millis();  // mark when we made the request to make sure we wait long enough before reading it.
    }

    Serial.print(tempA);
    Serial.print(" ");
    Serial.print(tempB);
    Serial.print(" ");
    Serial.print(tempC);
    Serial.print(" ");
    Serial.print(tempD);
    Serial.print(" ");
    Serial.print(tempE);
    Serial.print(" ");
    Serial.println(millis());
   
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
