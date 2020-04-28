#include "fileStuff.h"
#include<SoftwareSerial.h>

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  Serial.println(x[4]);
  Serial.println(xSize);
  Serial.println(aString);
  delay(10000);
}
