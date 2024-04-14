#include "EasyTransfer.h"
#include "SoftwareSerial.h"

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_PWMServoDriver.h>
#include "RemoteController_dataStructures.h"

#include <Math.h>

#include <Servo.h>
#include <Ramp.h>

#include "MovingAverage.h"

#include "MyLcd.h"
#include "MyLcd_SplashScreen.h"
#include "FormData.h"
#include "constants.h"


//create object
EasyTransfer ET1;   // send serial
EasyTransfer ET2;   // rec serial

//https://robojax.com/learn/arduino/?vid=robojax_PCA9685-V1
#define SERVO_MIN   135                // https://www.arduino.cc/reference/en/libraries/servo/writemicroseconds/
#define SERVO_MAX   615                // value of 135 is fully counter-clockwise, 615 is fully clockwise.
//----------------------------------
#define SERVO_MIN_Pinky_Right    135
#define SERVO_MAX_Pinky_Right   615

#define SERVO_MIN_Pinky_Center   135
#define SERVO_MAX_Pinky_Center   615

#define SERVO_MIN_Pinky_Left   135
#define SERVO_MAX_Pinky_Left   615
//---------------------------------
#define SERVO_MIN_Ring_Right   135
#define SERVO_MAX_Ring_Right   615

#define SERVO_MIN_Ring_Center   135
#define SERVO_MAX_Ring_Center   615

#define SERVO_MIN_Ring_Left   135
#define SERVO_MAX_Ring_Left   615
//----------------------------------
#define SERVO_MIN_Middle_Right   135
#define SERVO_MAX_Middle_Right   615

#define SERVO_MIN_Middle_Center   135
#define SERVO_MAX_Middle_Center   615

#define SERVO_MIN_Middle_Left   135
#define SERVO_MAX_Middle_Left   615
//------------------------------------
#define SERVO_MIN_Index_Right   135
#define SERVO_MAX_Index_Right   615

#define SERVO_MIN_Index_Center   135
#define SERVO_MAX_Index_Center   615

#define SERVO_MIN_Index_Left   135
#define SERVO_MAX_Index_Left   615
//----------------------------------
#define SERVO_MIN_Thumb_Right   135
#define SERVO_MAX_Thumb_Right   615

#define SERVO_MIN_Thumb_Center   135
#define SERVO_MAX_Thumb_Center   615

#define SERVO_MIN_Thumb_Rotate   135
#define SERVO_MAX_Thumb_Rotate   615

#define SERVO_MIN_Thumb_Left   135
#define SERVO_MAX_Thumb_Left   615


// knee calcs
#define DIGITLENGTH 330L    // length of each top/bottom leg
#define KNEEROD 180L       // length of push rod
#define KNEEROD2 94L        // other side of push rod triangle
#define KNEEACTANGLE 15L   // angle between actuator and upp leg joint

// Outcomment line below for HM-10, HM-19 etc
//#define HIGHSPEED   // Most modules are only 9600, although you can reconfigure this
#define EN_PIN_HIGH   // You can use this for HC-05 so you don't have to hold the small button on power-up to get to AT-mode

#ifdef HIGHSPEED
  #define Baud 38400   // Serial monitor
  #define BTBaud 38400 // There is only one speed for configuring HC-05, and that is 38400.
#else
  #define Baud 9600    // Serial monitor
  #define BTBaud 9600  // HM-10, HM-19 etc
#endif


#define STATE 11
#define BLUETOOTH_RX 9  // Bluetooth RX -> Arduino D9
#define BLUETOOTH_TX 10 // Bluetooth TX -> Arduino D10
//#define GND 13
//#define Vcc 12
#define ENABLE 8

/*
double legLength;           // required overall leg length
double kneeAngle;           // the actual angle of the knee between top and bottom sections
double kneeAngle2;          // angle between bottom of leg and actuator
double kneeAngle2a;          // angle between bottom of leg and actuator
double kneeAngle3;          // other angle
double kneeAngle3a;          // other angle
double kneeAngle4;          // other angle.
double kneeAngle4a;          // other angle
double kneeActuator;        // calculated length of actuator from joint
*/

int axis1;
int axis2;
int axis3;
int axis4;
int axis5;
int axis6;

int mode;
int count;
int noDataCount;

int menuFlag = 0;        

Servo servo01; //zakladna
Servo servo02; //spodne hnede rameno
Servo servo03; //horne  biela rameno
Servo servo04; //ruka nabok  100 = zhruba vodorovne

//MovingAverage<uint16_t,4> servo_MovingAverage01;
//MovingAverage<uint16_t,4> servo_MovingAverage02;
//MovingAverage<uint16_t,4> servo_MovingAverage03;
//MovingAverage<uint16_t,4> servo_MovingAverage04;

uint16_t servo01_Avg;
uint16_t servo02_Avg;
uint16_t servo03_Avg;
uint16_t servo04_Avg;

uint16_t servo01_constrained;
uint16_t servo02_constrained;
uint16_t servo03_constrained;
uint16_t servo04_constrained;

uint16_t servo01_Angle;
uint16_t servo02_Angle;
uint16_t servo03_Angle;

uint16_t servo04_Angle;
uint16_t servo05_Angle;
uint16_t servo06_Angle;
uint16_t servo07_Angle;

SEND_DATA_STRUCTURE mydata_send;
RECEIVE_DATA_STRUCTURE mydata_remote;

unsigned long previousMillis = 0;
const long interval = 20;

unsigned long previousServoMillis=0;
const long servoInterval = 200;

long previousSafetyMillis;

int state; // BT state
int previous_state;

byte showForm;

// Set the pins on the I2C chip used for LCD connections (Some LCD use Address 0x27 and others use 0x3F):
LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 20 chars and 4 line display
MyLcd myLcd;
MyLcd_SplashScreen myLcd_SplashScreen;

SoftwareSerial bluetooth(BLUETOOTH_TX, BLUETOOTH_RX);

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup() {

  myLcd.begin(lcd);

  Serial.begin(Baud, SERIAL_8N1);
  Serial.println(" ");
  Serial.print("Sketch:   ");   Serial.println(__FILE__);
  Serial.print("Uploaded: ");   Serial.println(__DATE__);

  BT_to_serial_prepare();

	showForm = form_SplashScreen;
  // NOTE: Cursor Position: (CHAR, LINE) starts at 0  
  
  lcd.backlight();
  Serial.println("setup: after lcd.backlight..");
  //delay(2000);
  
  Serial.println("setup: Before myLcd.showSplashScreen()..");
  myLcd.showSplashScreen();
  Serial.println("setup: After myLcd.showSplashScreen()..");
  //delay(500);
  //Init_PinModes();
  previous_state = 0;
  delay(1000);
  Serial.println("setup: before lcd.clear");
  lcd.clear();

  
  Serial.println("setup:: Servo Initialization started");
	delay(200);
  pwm.begin(); //pwm.begin(0);   0 = driver_ID
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
	Serial.println("setup: Servos on PCA9685  attached");
  delay(20);
  
}
//----------------------------BT_to_serial_prepare-----------------------------------------
void BT_to_serial_prepare() {

    Serial.println("Bluetooth initialization....");

    // Setup BT module
    pinMode(BLUETOOTH_TX, INPUT);
    pinMode(BLUETOOTH_RX, OUTPUT);  
    pinMode(STATE, INPUT);
    pinMode(ENABLE, OUTPUT);
    #ifdef EN_PIN_HIGH  
      digitalWrite(ENABLE, HIGH);   // Used to force AT-mode for HC-05. More flexible is to press the button on the pcb
    #endif
    
    bluetooth.begin(BTBaud);
    ET1.begin(details(mydata_send), &bluetooth);
    ET2.begin(details(mydata_remote), &bluetooth);
    //bluetooth_initialized = true;
    Serial.println("Bluetooth available.");
    //previous_Bluetooth_State = bluetooth_On;
    
}
//----------------------------end of BT_to_serial_prepare----------------------------------

void loop() {
   unsigned long currentMillis = millis();
       if (currentMillis - previousMillis >= interval) {  // start timed event for read and send
            previousMillis = currentMillis;

            if(ET2.receiveData()){                                        // main data receive
                previousSafetyMillis = currentMillis; 

                mydata_send.mode = mode;
                mydata_send.count = count;

                ET1.sendData();                                           // send data back to remote       
              /*  
                Serial.println( "LX:"+String(mydata_remote.stick1_X)+
                              ", LY:"+String(mydata_remote.stick1_Y)+
                              ", RX:"+String(mydata_remote.stick2_X)+
                              ", RY:"+String(mydata_remote.stick2_Y)+
                              ", count:"+String(count));
              */                
              //if(showForm == form_ShowMeasuredData){

                String btnsString = "";//"Btn"+String(mydata_remote.menuDown)+""+String(mydata_remote.menuUp)+""+String(mydata_remote.Select)+""+String(mydata_remote.toggleBottom)+""+String(mydata_remote.toggleTop)+""+"X";
                //btnsString = btnsString +" Nav"+ String(mydata_remote.navKeyUp)+""+String(mydata_remote.navKeyDown)+""+String(mydata_remote.navKeyLeft)+""+String(mydata_remote.navKeyRight)+""+String(mydata_remote.navKeyMiddle)+""+String(mydata_remote.navKeySet)+""+String(mydata_remote.navKeyReset);

                myLcd.showMeasuredDateScreen(mydata_remote.stick1_X, mydata_remote.stick2_X, mydata_remote.stick1_Y, mydata_remote.stick2_Y, btnsString, "count:"+String(count)+" mode:"+String(mode));
                //myLcd.showMeasuredDateScreen2(leftJoystick_X,leftJoystick_Y, rightJoystick_X, rightJoystick_Y, mydata_send.index_finger_knuckle_right, mydata_send.pinky_knuckle_right, mydata_send.index_finger_fingertip,mydata_send.index_finger_knuckle_left, btnsString, "");
              //}

              servo01_constrained = constrain(mydata_remote.stick1_X, 0, 1023);
              servo02_constrained = constrain(mydata_remote.stick1_Y, 0, 1023);
              servo03_constrained = constrain(mydata_remote.stick2_X, 0, 1023);
              servo04_constrained = constrain(mydata_remote.stick2_Y, 0, 1023);

              servo01_constrained = map(servo01_constrained, 1023, 0, SERVO_MIN, SERVO_MAX);  //Inverted
              servo02_constrained = map(servo02_constrained, 1023, 0, SERVO_MIN, SERVO_MAX);  //Inverted
              servo03_constrained = map(servo03_constrained, 0, 1023, SERVO_MIN, SERVO_MAX);
              servo04_constrained = map(servo04_constrained, 0, 1023, SERVO_MIN, SERVO_MAX);  //Inverted


              servo01_Angle = (servo01_constrained + servo02_constrained)/2;
              servo02_Angle =  servo04_constrained;
              servo03_Angle = (servo01_constrained + (1023 - servo02_constrained))/2;

              servo04_Angle = map(servo01_Angle, 0, 1023, SERVO_MIN, SERVO_MAX);
              servo05_Angle = map(servo02_Angle, 0, 1023, SERVO_MIN, SERVO_MAX);
              servo06_Angle = map(servo03_Angle, 0, 1023, SERVO_MIN, SERVO_MAX);
              servo07_Angle = map(servo01_Angle, 0, 1023, SERVO_MIN, SERVO_MAX);

              //servo08_Angle = map(servo02_Angle, 0, 1023, SERVO_MIN, SERVO_MAX);
              //servo09_Angle = map(servo03_Angle, 0, 1023, SERVO_MIN, SERVO_MAX);
              

              /*
              Serial.println(   "LX:"+String(mydata_remote.stick1_X)+ ", S1:" + String(servo01_Angle) +
                            ",   LY:"+String(mydata_remote.stick1_Y       )+ ", S2:" + String(servo02_Angle) +
                            ",   RX:"+String(mydata_remote.stick2_X    )+ ", S3:" + String(servo03_Angle) +
                            ",   RY:"+String(mydata_remote.stick2_Y )+ ", S4:" + String(servo04_Angle) +
                            ", count:"+String(count));
              */
             // end of receive data

              count = count+1;                                              // update count for remote monitoring
              /*
              lcd.setCursor(0,2);
              lcd.print(count);
              lcd.setCursor(0,3);
              lcd.print("Mode - ");
              lcd.setCursor(7,3);
              lcd.print(mode);
              */
            } else if(currentMillis - previousSafetyMillis > 200) {         // safeties
              noDataCount = noDataCount+1;                                              // update count for remote monitoring
              lcd.setCursor(0,0);
              lcd.print("!"+String(noDataCount)+"! No Data ");
              //Serial.println("No Data");
              //lcd.print(" No Data ");
            }
            //count = count+1;                                              // update count for remote monitoring

            
       }  // end of timed event Receive/Send

      if (currentMillis - previousServoMillis >= servoInterval) {  // start timed event for Servos  (200 ms)
        previousServoMillis = currentMillis;

        
        pwm.setPWM(15, 0, map(servo01_Angle, 0, 1023, SERVO_MIN_Pinky_Right, SERVO_MAX_Pinky_Right));  //Servo 0
        pwm.setPWM(14, 0, map(servo02_Angle, 0, 1023, SERVO_MIN_Pinky_Center, SERVO_MAX_Pinky_Center));  //Servo 1
        pwm.setPWM(13, 0, map(servo03_Angle, 0, 1023, SERVO_MIN_Pinky_Left, SERVO_MAX_Pinky_Left));  //Servo 2

        pwm.setPWM(12, 0, map(servo01_Angle, 0, 1023, SERVO_MIN_Ring_Right, SERVO_MAX_Ring_Right));  //Servo 0
        pwm.setPWM(11, 0, map(servo02_Angle, 0, 1023, SERVO_MIN_Ring_Center, SERVO_MAX_Ring_Center));  //Servo 1
        pwm.setPWM(10, 0, map(servo03_Angle, 0, 1023, SERVO_MIN_Ring_Left, SERVO_MAX_Ring_Left));  //Servo 2

        pwm.setPWM( 9, 0, map(servo01_Angle, 0, 1023, SERVO_MIN_Middle_Right, SERVO_MAX_Middle_Right));  //Servo 0
        pwm.setPWM( 8, 0, map(servo02_Angle, 0, 1023, SERVO_MIN_Middle_Center, SERVO_MAX_Middle_Center));  //Servo 1
        pwm.setPWM( 7, 0, map(servo03_Angle, 0, 1023, SERVO_MIN_Middle_Left, SERVO_MAX_Middle_Left));  //Servo 2

        pwm.setPWM( 6, 0, map(servo01_Angle, 0, 1023, SERVO_MIN_Index_Right, SERVO_MAX_Index_Right));  //Servo 0
        pwm.setPWM( 5, 0, map(servo02_Angle, 0, 1023, SERVO_MIN_Index_Center, SERVO_MAX_Index_Center));  //Servo 1
        pwm.setPWM( 4, 0, map(servo03_Angle, 0, 1023, SERVO_MIN_Index_Left, SERVO_MAX_Index_Left));  //Servo 2

        pwm.setPWM( 3, 0, map(servo04_Angle, 0, 1023, SERVO_MIN_Thumb_Right, SERVO_MAX_Thumb_Right));  //Servo 3
        pwm.setPWM( 2, 0, map(servo05_Angle, 0, 1023, SERVO_MIN_Thumb_Center, SERVO_MAX_Thumb_Center));  //Servo 4
        pwm.setPWM( 1, 0, map(servo06_Angle, 0, 1023, SERVO_MIN_Thumb_Rotate, SERVO_MAX_Thumb_Rotate));  //Servo 3
        pwm.setPWM( 0, 0, map(servo07_Angle, 0, 1023, SERVO_MIN_Thumb_Left, SERVO_MAX_Thumb_Left));  //Servo 4
        
      }

}

