#include "SCServo.h"
#include <Math.h>
#include <Arduino.h>
#include <Stream.h>

#define Baud 9600    // Serial monitor
SCSCL sc;
SCServo SERVO;      //Declare a case of SCServo to control the Feetechs


void setup()
{
  Serial.begin(Baud, SERIAL_8N1);
  Serial.println(" ");
  Serial.print("Sketch:   ");   Serial.println(__FILE__);
  Serial.print("Uploaded: ");   Serial.println(__DATE__);


  Serial.println("Setup: Serial.begin(..)");
  Serial2.begin(1000000);
  Serial.println("Setup: sc.Serial");
  //sc.pSerial = &Serial2;
  SERVO.pSerial = &Serial2;
  Serial.println("Setup: sc.WritePos(..)");
  //sc.WritePos(0xfe, 0, 1000);
  SERVO.WritePos(0xfe, 0, 1000);
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
    Serial.print("loop: Up. i=");
    Serial.println(i);
    //sc.WritePos(0xfe, i, 20);
    SERVO.WritePos(0xfe, i, 20);
    delay(20);
  }
  Serial.println("loop: Down");
  for(i=730; i>100; i-=10)
  {
    Serial.print("loop: Down. i=");
    Serial.println(i);
    //sc.WritePos(0xfe, i, 20);
    SERVO.WritePos(0xfe, i, 20);
    delay(20);
  }
    Serial.println("loop: End. OK");

}
