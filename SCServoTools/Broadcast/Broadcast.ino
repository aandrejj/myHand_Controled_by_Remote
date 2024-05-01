#include "SCServo.h"
#include <Math.h>
#include <Arduino.h>
#include <Stream.h>

//SCSCL sc;
SCServo SERVO;      //Declare a case of SCServo to control the Feetechs


void setup()
{
  Serial.println("Setup: Serial.begin(..)");
  Serial2.begin(1000000);
  Serial.println("Setup: sc.Serial");
  sc.pSerial = &Serial2;
  Serial.println("Setup: sc.WritePos(..)");
  sc.WritePos(0xfe, 0, 1000);
  Serial.println("Setup: delay");
  delay(1000);        
  Serial.println("Setup: Ok. End");
}

void loop()
{
  Serial.println("loop: Start");
  int i;
  for(i=100; i<730; i+=10)
  {
    Serial.println("loop": Up. i="+String(i));
    sc.WritePos(0xfe, i, 20);
    delay(20);
  }
  Serial.println("loop": Down");
  for(i=730; i>100; i-=10)
  {
    Serial.println("loop": Down. i="+String(i));
    sc.WritePos(0xfe, i, 20);
    delay(20);
  }
    Serial.println("loop": End. OK");

}
