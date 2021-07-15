#include <SoftwareSerial.h>
SoftwareSerial softSerial(10, 11);
int event;

void setup()  
{
  softSerial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
} 
void loop()  
{ 
  event = random(0,2);
  if(event){
    softSerial.write("Go");
  }
  digitalWrite(LED_BUILTIN, HIGH);
  delay (100);
  digitalWrite(LED_BUILTIN, LOW);
  delay (100);
}
