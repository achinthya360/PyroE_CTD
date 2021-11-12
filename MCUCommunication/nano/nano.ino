// REQUIRED LIBRARIES
#include <Servo.h>
#include <AccelStepper.h>

// CONSTANTS AND VARIABLE DECLARATIONS

// MCU COMMUNICATION
const int trig = 2;
bool wake = false;

// LEAK SENSOR
int leakPin = 9;
bool leak = false;

// WATER LEVEL SENSOR
int wetPin = 8;
bool wet = false;

// SHAFT ANGLE SENSOR
int shaftAnglePin = 7;
bool shaftAngle;

// MOTORS
int bs1ONpin = 5;
int bs2ONpin = 4;
int bs1controlPin = A0;
int bs2controlPin = A1;
Servo ballast1;
Servo ballast2;

// 48V TAIL MOTOR CONTROL
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

// Define a stepper and the pins it will use
AccelStepper stepper(AccelStepper::DRIVER, 3, 2); // Using stepper pins (3 for pulse, 2 for dir)


void setup()
{
  // only for testing with computer
  Serial.begin(9600);

  // MCU COMMUNICATION

  //  softSerial.begin(9600);
  //  Serial1.begin(9600);
  //  pinMode(LED_BUILTIN, OUTPUT);
  //  pinMode(trig, OUTPUT);
  //  digitalWrite(trig, HIGH);
  //  digitalWrite(LED_BUILTIN, HIGH);



  // SENSOR INTEGRATION

  pinMode(leakPin, INPUT);
  pinMode(wetPin, INPUT);
  pinMode(shaftAnglePin, INPUT);



  // MOTOR CONTROL
  pinMode(bs1ONpin, OUTPUT);
  digitalWrite(bs2ONpin, LOW);
  pinMode(bs2ONpin, OUTPUT);
  digitalWrite(bs2ONpin, LOW);
  ballast1.attach(bs1controlPin);
  ballast2.attach(bs2controlPin);
  // 48V TAIL MOTOR CONTROL
  stepper.setMaxSpeed(max_speed);
  stepper.setAcceleration(a);
  // this function should center the tail to 0 degrees (may need slight degree offset)
  centerTailMotor();

  // initiate first sweep of motor
  stepper.moveTo(amplitude);
}
void loop()
{
  // MCU COMMUNICATION

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



  // SENSOR INTEGRATION

  // monitor leaking based on 0 or 1 from leak sensor
  leak = (digitalRead(leakPin) == 1) ? true : false;

  // monitor if wet or not using water level sensor
  wet = (digitalRead(wetPin) == 1) ? true : false;

  // monitor if tail is centered
  shaftAngle = (digitalRead(shaftAnglePin) == 1) ? true : false;



  // MOTOR CONTROL
  //  ballast1.write(//insert wanted position here);
  //  ballast2.write(//insert wanted position here);
  // 48V TAIL MOTOR CONTROL
  // If at the end of travel go to the other end
  if (stepper.distanceToGo() == 0)
    stepper.moveTo(-stepper.currentPosition());
  stepper.run();



  // TESTING WITH COMPUTER
  Serial.print("Leak: ");
  Serial.print(leak);
  Serial.print("\tWet: ");
  Serial.print(wet);
  Serial.print("\tShaft Angle: ");
  Serial.println(shaftAngle);
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
