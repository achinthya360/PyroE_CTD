#include <Arduino.h>   // required before wiring_private.h
#include "wiring_private.h" // pinPeripheral() function

// SD card peripherals
#include <SD.h>
File root;

Uart Serial2 (&sercom1, 11, 10, SERCOM_RX_PAD_0, UART_TX_PAD_2);

// SD card CS
int chipSelect = 6;

String str = "";

void SERCOM1_Handler()
{
  Serial2.IrqHandler();
}

void setup() {
  // SD card setup with HW-125
  pinMode(chipSelect, OUTPUT);
  SD.begin(chipSelect);
  root = SD.open("/");
  
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
//  Serial2.write(i++);
  if (Serial2.available()) {
    uint8_t data = Serial2.read();
//    Serial.print(" -> 0x"); 
    Serial.print(data);
    str += (char)data;
  }
//  Serial.println();
  if(str == "L"){
    printDirectory(root, 0);
    str = "";
  }
  if(str == "hello"){
    Serial.println("DUB!");
  }
  delay(10);
}

void printDirectory(File dir, int numTabs)
{
  String entryname = "";
  while (true)
  {
    File entry = dir.openNextFile();
    if (! entry)
    {
      if (numTabs == 0)
        Serial2.println("** Done **");
      return;
    }
    for (uint8_t i = 0; i < numTabs; i++)
      Serial2.print('\t');
   entryname = entry.name();
    if(!entryname.startsWith("LOG")){
      continue;
    }
    Serial.println(entryname);
    Serial2.print(entryname);
    File dataFile = SD.open(entryname);

  // if the file is available, write to it:
  if (dataFile) {
    while (dataFile.available()) {
      delay(10);
      Serial2.write(dataFile.read());
    }
    dataFile.close();
  }
    delay(10);
//    if (entry.isDirectory())
//    {
//      Serial2.println("/");
//      printDirectory(entry, numTabs + 1);
//    }
//    else
//    {
//      Serial2.print("\t\t");
//      Serial2.println(entry.size(), DEC);
//    }
    entry.close();
  }
}
