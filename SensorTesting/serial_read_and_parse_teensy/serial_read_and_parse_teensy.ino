
//example code on how to use the teensy to read the IMU data
//perform necessary computation as needed

// Buffer to store incoming commands from serial port
String str;
double ax, ay, az;
double q0,q1,q2,q3;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); //this is the USB
  Serial1.begin(115200); //using TX1, RX1, pin 0 and pin1
  //init the data output from IMU
  Serial1.println("*");
}

void loop() {
  // put your main code here, to run repeatedly:
   if(Serial1.available() > 0) {
     str = Serial1.readStringUntil('\n');
     Serial.println(str);//print out to USB to debug
     //convert to float
     
     char charBuf[str.length() + 1];
     str.toCharArray(charBuf, str.length());
     int result = sscanf(charBuf, "%lf,%lf,%lf,%lf,%lf,%lf,%lf\n", &ax, &ay, &az, &q0, &q1, &q2, &q3);

     //reset str
     str="";
   }//end if read serial
     
     
   //do something with the data
     
}
