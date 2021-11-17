const byte ledPin = 13;
const byte interruptPin = 2;
volatile int magSwitchEvent;
// keeps track if a magSwitchEvent currently happening or not
volatile byte magFlag;

void setup() {
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  pinMode(interruptPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin), magSwitchFlick, RISING);
  Serial.begin(9600);
  magFlag = false;
}

void loop() {
  Serial.println(digitalRead(interruptPin));
  // if a magSwitchEvent is occurring and 3 seconds have passed
  if(magFlag && ((millis() - magSwitchEvent) > 3000)){
    // if the magswitch is still ON
    if(digitalRead(interruptPin) == HIGH){
      // either turn 48V motor ON or OFF based on previous state
      digitalWrite(ledPin, !digitalRead(ledPin));
    }
    // let MCU know magSwitchEvent is over/resolved
    magFlag = false;
  }
}

void magSwitchFlick() {
  Serial.println("mag Switch detected");
  magFlag = true;
  magSwitchEvent = millis();
}
