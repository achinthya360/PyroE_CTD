#include <SD.h>

File root;

String filetodelete;

void setup()
{
  Serial.begin(9600);
  pinMode(4, OUTPUT);
  SD.begin(4);
  root = SD.open("/");
  wipeDirectory(root);
  delay(2000);
  Serial.println();
  Serial.println("Rewinding, and repeating below:" );
  Serial.println();
  delay(2000);
  root.rewindDirectory();
  printDirectory(root, 0);
  root.close();
}

void loop()
{
  // nothing happens after setup finishes.
}

void printDirectory(File dir, int numTabs)
{
  while (true)
  {
    File entry = dir.openNextFile();
    if (! entry)
    {
      if (numTabs == 0)
        Serial.println("** Done **");
      return;
    }
    for (uint8_t i = 0; i < numTabs; i++)
      Serial.print('\t');
    Serial.print(entry.name());
    if (entry.isDirectory())
    {
      Serial.println("/");
      printDirectory(entry, numTabs + 1);
    }
    else
    {
      Serial.print("\t\t");
      Serial.println(entry.size(), DEC);
    }
    entry.close();
  }
}

void wipeDirectory(File dir)
{
  while (true)
  {
    File entry = dir.openNextFile();
    if (! entry)
    {
      return;
    }
    else
    {
      filetodelete = entry.name();
    }
    entry.close();
    SD.remove(filetodelete);
    Serial.println("Deleted file");
  }
}
