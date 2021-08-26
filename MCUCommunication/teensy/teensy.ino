//SoftwareSerial softSerial(10, 11);
int event;

const int trig = 2;
bool wake = false;

void setup()  
{
//  softSerial.begin(9600);
  Serial1.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(trig, OUTPUT);
  digitalWrite(trig, HIGH);
  digitalWrite(LED_BUILTIN, HIGH);
} 
void loop()  
{ 
//  event = random(0,2);
//  if(event){
//    softSerial.write("Go");
//  }
//  digitalWrite(LED_BUILTIN, HIGH);
//  delay (100);
//  digitalWrite(LED_BUILTIN, LOW);
//  delay (100);
  if(!wake and Serial1.read() == 'o'){
    pinMode(trig, OUTPUT); // connect and pull Feather's ENable pin to GND
    digitalWrite(trig, LOW);
    digitalWrite(LED_BUILTIN, LOW);
    delay(20000);
    wake = true;
  }
  if(wake){
    pinMode(trig, INPUT); // disconnect and let Feather's internal pullup turn on ENable pin
//    digitalWrite(trig, HIGH);
    digitalWrite(LED_BUILTIN, HIGH);
    wake = false;
  }
}
