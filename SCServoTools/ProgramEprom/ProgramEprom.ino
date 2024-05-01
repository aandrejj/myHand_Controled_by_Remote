#include <SCServo.h>

int LEDpin = 13;
SCSCL sc;

void setup()
{
  pinMode(LEDpin, OUTPUT);
  Serial1.begin(1000000); //115200);
  sc.pSerial = &Serial1;
  digitalWrite(LEDpin, LOW);
  sc.writeByte(0xfe, SCSCL_LOCK, 0);//Turn on EPROM saving function
  sc.writeByte(0xfe, SCSCL_ID, 2);//ID
  delay(50);
  sc.writeWord(0xfe, SCSCL_MIN_ANGLE_LIMIT_L, 20);
  delay(50);
  sc.writeWord(0xfe, SCSCL_MAX_ANGLE_LIMIT_L, 1023);
  delay(50);
  sc.writeWord(0xfe, SCSCL_MAX_TORQUE_L, 1000);
  delay(50);
  sc.writeWord(0xfe, SCSCL_PUNCH_L, 16);
  delay(50);
  sc.writeByte(0xfe, SCSCL_BAUD_RATE, SCSCL_1000000);//SCSCL_115200);
  delay(50);
  sc.writeByte(0xfe, SCSCL_LOCK, 1);//Turn off EPROM saving function
  digitalWrite(LEDpin, HIGH);
}

void loop()
{

}
