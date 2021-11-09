// Constants and declarations
const int trig = 2;
bool wake = false;

// LEAK SENSOR
int leakPin = 5;
bool leak = false;

// WATER LEVEL SENSOR
int wetPin = 9;
bool wet = false;

// SHAFT ANGLE SENSOR
int shaftAnglePin = 7;
bool shaftAngle; 


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
  leak = (digitalRead(leakPin)==1) ? true : false;

  // monitor if wet or not using water level sensor
  wet = (digitalRead(wetPin) == 1) ? true : false;

  // monitor if tail is centered
  shaftAngle = (digitalRead(shaftAnglePin) == 1) ? true : false;


  // TESTING WITH COMPUTER
  Serial.print("Leak: ");
  Serial.print(leak);
  Serial.print("\tWet: ");
  Serial.print(wet);
  Serial.print("\tShaft Angle: ");
  Serial.println(shaftAngle);
}
