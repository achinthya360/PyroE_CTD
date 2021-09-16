#include <Arduino.h>   // required before wiring_private.h
#include "wiring_private.h" // pinPeripheral() function

Uart Serial2 (&sercom1, 11, 10, SERCOM_RX_PAD_0, UART_TX_PAD_2);

String str = "";

void SERCOM1_Handler()
{
  Serial2.IrqHandler();
}

void setup() {
  Serial.begin(9600);

  Serial2.begin(9600);
  
  // Assign pins 10 & 11 SERCOM functionality
  pinPeripheral(10, PIO_SERCOM);
  pinPeripheral(11, PIO_SERCOM);

  str = "";
}

uint8_t i=0;
void loop() {
  
//  Serial.print(i);
  Serial2.write(i++);
  if (Serial2.available()) {
    uint8_t data = Serial2.read();
    Serial.print(" -> 0x"); Serial.print(data, HEX);
    str += (char)data;
  }
//  Serial.println();
  if(str == "hello\n"){
    Serial.println("DUB!");
    str = "";
  }
  delay(10);
}
