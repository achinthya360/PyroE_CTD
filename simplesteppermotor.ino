/*Example sketch to control a stepper motor with A4988 stepper motor driver and Arduino without a library. More info: https://www.makerguides.com */

// Constants for updateInterval sinusoid math
int amplitude = 200;
float pi = 3.14;
float frequency = 9;


// Define stepper motor connections and steps per revolution:
#define dirPin 2
#define stepPin 3
#define stepsPerRevolution 1600

#define potPin A0

int interval;

// true if using potentiometer, false if using sinusoid
bool pot = false;

void setup() {
  // Declare pins as output:
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(A0, INPUT);
}

void loop() {
  if (pot) {
    interval = analogRead(potPin) * 150/1024;
  }
  else {
    interval = (int) ((sin(frequency * millis()/1000) * amplitude));
    if (interval >= 0){
      digitalWrite(dirPin, HIGH); //CCW is HIGH
      interval = -(interval - amplitude); // invert sinusoid to interval
    }
    else{
      digitalWrite(dirPin, LOW); //CW is LOW
      interval += amplitude; // invert sinusoid to interval
    }
  }
  delayMicroseconds(5); // 3 microsecond delay from digitalWrite in updateInterval + 2 = 5

  digitalWrite(stepPin, HIGH);
  delayMicroseconds(interval);
  digitalWrite(stepPin, LOW);
}
