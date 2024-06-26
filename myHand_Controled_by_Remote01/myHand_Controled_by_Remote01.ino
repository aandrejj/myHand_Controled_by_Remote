#include "EasyTransfer.h"
#include "SoftwareSerial.h"
#include "SCServo.h"

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
#include "Servo_Min_Max.h"
#include "myHand_Controled_by_Remote.h"

//create object
EasyTransfer ET1;   // send serial
EasyTransfer ET2;   // rec serial

//https://robojax.com/learn/arduino/?vid=robojax_PCA9685-V1

//SCServo SERVO;      //Declare a case of SCServo to control the Feetechs
SCSCL sc;

int axis1;
int axis2;
int axis3;
int axis4;
int axis5;
int axis6;

int16_t mode;
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
uint16_t servo05_constrained;
uint16_t servo06_constrained;
uint16_t servo07_constrained;
uint16_t servo08_constrained;

uint16_t servo01_Angle;
uint16_t servo02_Angle;
uint16_t servo03_Angle;

uint16_t servo04_Angle;
uint16_t servo05_Angle;
uint16_t servo06_Angle;
uint16_t servo07_Angle;
uint16_t servo08_Angle;

SEND_DATA_STRUCTURE mydata_send;
RECEIVE_DATA_STRUCTURE mydata_remote;

unsigned long previousMillis = 0;
const long interval = 20;

unsigned long previousServoMillis=0;
const long servoInterval = 300;

long previousSafetyMillis;

int state; // BT state
int previous_state;

int wrist_pos[] = {512, 512, 512};     // default to centre

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

  SCServoInitialize();

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
  tmp_mode = 0;
  previous_mode = tmp_mode;  
}
//----------------------------SCServo initialization---------------------------------------
void SCServoInitialize() {
  Serial.println("SCServoInitialize: started");
  Serial2.begin(1000000);     //Comms with Feetech servos RX2 = D17, TX2 = D16                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          
  sc.pSerial = &Serial2;
  Serial.println("SCServoInitialize: End");
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

                //mydata_send.mode = mode;
                mydata_send.count = count;


                ET1.sendData();                                           // send data back to remote       

              servo01_constrained = constrain(mydata_remote.stick1_X, 0, 1023);
              servo02_constrained = constrain(mydata_remote.stick1_Y, 0, 1023);
              servo03_constrained = constrain(mydata_remote.stick2_X, 0, 1023);
              servo04_constrained = constrain(mydata_remote.stick2_Y, 0, 1023);

              servo05_constrained = constrain((mydata_remote.stick3_X - 200), 0, 1023);
              servo06_constrained = constrain((mydata_remote.stick3_Y + 300), 0, 1023);
              servo07_constrained = constrain(mydata_remote.stick4_X, 0, 1023);
              servo08_constrained = constrain(mydata_remote.stick4_Y, 0, 1023);

              servo01_constrained = map(servo01_constrained, 1023, 0, 0, 1023);  //Inverted
              servo02_constrained = map(servo02_constrained, 1023, 0, 0, 1023);  //Inverted
              servo03_constrained = map(servo03_constrained, 0, 1023, 0, 1023);
              servo04_constrained = map(servo04_constrained, 0, 1023, 0, 1023);

              servo05_constrained = map(servo05_constrained, 0, 1023, 0, 1023);  //Inverted
              servo06_constrained = map(servo06_constrained, 0, 1023, 0, 1023);  //Inverted
              servo07_constrained = map(servo07_constrained, 0, 1023, 0, 1023);
              servo08_constrained = map(servo08_constrained, 0, 1023, 0, 1023);

              servo01_Angle = (servo01_constrained + servo02_constrained)/2;  //Left
              servo02_Angle =  servo04_constrained; //Center
              servo03_Angle = (servo01_constrained + (1023 - servo02_constrained))/2; //Right
              servo04_Angle =  servo03_constrained;  //Tumb Rotation
              
             //angles[16] = constrain(angles[16], 70, 110);  //servo05_Angle
             //angles[17] = constrain(angles[17], 25, 65);   //servo06_Angle
             //angles[18] = constrain(angles[18], 70, 115);  //servo07_Angle
              
              //servo05_Angle = map(servo05_constrained, 0, 1023, 70, 110);
              //servo06_Angle = map(servo06_constrained, 0, 1023, 25, 65);
              //servo07_Angle = map(servo07_constrained, 0, 1023, 70, 115);
              //servo08_Angle = map(servo08_constrained, 0, 1023, SERVO_MIN, SERVO_MAX);

              servo05_Angle = map((servo05_constrained + servo06_constrained)/2, 0, 1023, 1023, 23);
              servo06_Angle = map((servo05_constrained + (1023 - servo06_constrained))/2, 0, 1023, 1023, 23);
              servo07_Angle = map(servo07_constrained, 0, 1023, 23, 1023);
              servo08_Angle = map(servo08_constrained, 0, 1023, SERVO_MIN, SERVO_MAX);

              previous_mode = tmp_mode;

              mode = mydata_remote.mode;
              tmp_mode = mode;

              if(tmp_mode>= 32) {
                switch6Up = 0; 
                tmp_mode = tmp_mode - 32;
              } else {
                switch6Up = 1;
              }

              if(tmp_mode>= 16) {
                switch5Up = 0; 
                tmp_mode = tmp_mode - 16;
              } else {
                switch5Up = 1;
              }

              if(tmp_mode>= 8) {
                switch4Up = 0; 
                tmp_mode = tmp_mode - 8;
              } else {
                switch4Up = 1;
              }

              if(tmp_mode>= 4) {
                switch3Up = 0; 
                tmp_mode = tmp_mode - 4;
              } else {
                switch3Up = 1;
              }

              if(tmp_mode>= 2) {
                switch2Up = 0; 
                tmp_mode = tmp_mode - 2;
              } else {
                switch2Up = 1;
              }

              if(tmp_mode>= 1) {
                switch1Up = 0; 
                tmp_mode = tmp_mode - 1;
              } else {
                switch1Up = 1;
              }

              if(previous_mode != tmp_mode) {
                Serial.println("New  tmp_mode ="+String(tmp_mode));
              }

              // end of receive data

              //if(showForm == form_ShowMeasuredData){
                //String btnsString = "L"+String(map(servo01_Angle, 0, 1023, SERVO_MIN_Thumb_Left, SERVO_MAX_Thumb_Left))+",C"+String(map(servo02_Angle, 0, 1023, SERVO_MIN_Thumb_Center, SERVO_MAX_Thumb_Center))+",R"+String(map(servo04_Angle, 0, 1023, SERVO_MIN_Thumb_Right, SERVO_MAX_Thumb_Right))+",T"+map(servo03_Angle, 0, 1023, SERVO_MIN_Thumb_Rotate, SERVO_MAX_Thumb_Rotate);
                String btnsString = "5:"+String(servo05_Angle) + ", 6:"+String(servo06_Angle) + ", 7:"+String(servo07_Angle) + ".";
                
                //String btnsString = "SL:"+String(map(servo03_Angle, 0, 1023, SERVO_MIN_Index_Left, SERVO_MAX_Index_Left))+",SC:"+String(map(servo02_Angle, 0, 1023, SERVO_MAX_Index_Center, SERVO_MIN_Index_Center))+",SR:"+String(map(servo01_Angle, 0, 1023, SERVO_MIN_Index_Right, SERVO_MAX_Index_Right));
                //String btnsString = "SL:"+String(map(servo03_Angle, 0, 1023, SERVO_MIN_Middle_Left, SERVO_MAX_Middle_Left))+",SC:"+String(map(servo02_Angle, 0, 1023, SERVO_MAX_Middle_Center, SERVO_MIN_Middle_Center))+",SR:"+String(map(servo01_Angle, 0, 1023, SERVO_MIN_Middle_Right, SERVO_MAX_Middle_Right));
                //btnsString = btnsString +" Nav"+ String(mydata_remote.navKeyUp)+""+String(mydata_remote.navKeyDown)+""+String(mydata_remote.navKeyLeft)+""+String(mydata_remote.navKeyRight)+""+String(mydata_remote.navKeyMiddle)+""+String(mydata_remote.navKeySet)+""+String(mydata_remote.navKeyReset);

                //String btnsString = "SwU"+String(switch1Up)+String(switch2Up)+String(switch3Up)+""+String(switch4Up)+String(switch5Up)+" SwD"+ String(switch1Down)+""+String(switch2Down)+""+String(switch3Down)+""+String(switch4Down)+String(switch5Down);


                myLcd.showMeasuredDateScreen(mydata_remote.stick3_X, mydata_remote.stick4_X, mydata_remote.stick3_Y, mydata_remote.stick4_Y, btnsString, "count:"+String(count)+" mode:"+String(mode)+" ");
                //myLcd.showMeasuredDateScreen2(leftJoystick_X,leftJoystick_Y, rightJoystick_X, rightJoystick_Y, mydata_send.index_finger_knuckle_right, mydata_send.pinky_knuckle_right, mydata_send.index_finger_fingertip,mydata_send.index_finger_knuckle_left, btnsString, "");
              //}
              count = count+1;                                              // update count for remote monitoring

            } else if(currentMillis - previousSafetyMillis > 350) {         // safeties
              noDataCount = noDataCount+1;                                              // update count for remote monitoring
              lcd.setCursor(0,0);
              lcd.print("!"+String(noDataCount)+"! No Data ");
            }
            //count = count+1;                                              // update count for remote monitoring
       }  // end of timed event Receive/Send

      if (currentMillis - previousServoMillis >= servoInterval) {  // start timed event for Servos  (200 ms)
        previousServoMillis = currentMillis;
        
        if(switch1Up==0) {
          pwm.setPWM(15, 0, map(servo01_Angle, 0, 1023, SERVO_MIN_Pinky_Left, SERVO_MAX_Pinky_Left));  //
          pwm.setPWM(14, 0, map(servo02_Angle, 0, 1023, SERVO_MIN_Pinky_Center, SERVO_MAX_Pinky_Center));  //
          pwm.setPWM(13, 0, map(servo03_Angle, 0, 1023, SERVO_MIN_Pinky_Right, SERVO_MAX_Pinky_Right));  //
        }

        if(switch2Up==0) {
          pwm.setPWM(12, 0, map(servo01_Angle, 0, 1023, SERVO_MIN_Ring_Left, SERVO_MAX_Ring_Left));  //
          pwm.setPWM(11, 0, map(servo02_Angle, 0, 1023, SERVO_MIN_Ring_Center, SERVO_MAX_Ring_Center));  //
          pwm.setPWM(10, 0, map(servo03_Angle, 0, 1023, SERVO_MIN_Ring_Right, SERVO_MAX_Ring_Right));  //
        }
        
        if(switch3Up==0) {
          pwm.setPWM( 9, 0, map(servo01_Angle, 0, 1023, SERVO_MIN_Middle_Left, SERVO_MAX_Middle_Left));  //
          pwm.setPWM( 8, 0, map(servo02_Angle, 0, 1023, SERVO_MAX_Middle_Center, SERVO_MIN_Middle_Center));  //Inverted here
          pwm.setPWM( 7, 0, map(servo03_Angle, 0, 1023, SERVO_MIN_Middle_Right, SERVO_MAX_Middle_Right));  //
        }
        
        if(switch4Up==0) {
          pwm.setPWM( 6, 0, map(servo01_Angle, 0, 1023, SERVO_MIN_Index_Left, SERVO_MAX_Index_Left));  //Servo 0
          pwm.setPWM( 5, 0, map(servo02_Angle, 0, 1023, SERVO_MAX_Index_Center, SERVO_MIN_Index_Center));  //Inverted here
          pwm.setPWM( 4, 0, map(servo03_Angle, 0, 1023, SERVO_MIN_Index_Right, SERVO_MAX_Index_Right));  //Servo 2
        }
        
        if(switch5Up==0) {
          pwm.setPWM( 3, 0, map(servo03_Angle, 0, 1023, SERVO_MAX_Thumb_Left, SERVO_MIN_Thumb_Left));  //Servo 3
          pwm.setPWM( 2, 0, map(servo02_Angle, 0, 1023, SERVO_MIN_Thumb_Center, SERVO_MAX_Thumb_Center));  //Servo 4
          pwm.setPWM( 1, 0, map(servo04_Angle, 0, 1023, SERVO_MIN_Thumb_Rotate, SERVO_MAX_Thumb_Rotate));  //Servo 3
          pwm.setPWM( 0, 0, map(servo01_Angle, 0, 1023, SERVO_MIN_Thumb_Right, SERVO_MAX_Thumb_Right));  //Servo 4
        }
        
        if(switch6Up==0) {
        //if(1==2) { 
          // angles[16] is wrist pitch, range 70 to 110, centre 90
          // angles[17] is wrist yaw, range 25 to 65, centre 45
          // angles[18] is wrist rotation around long axis of forearm, range 70 to 110, centre 90
          
          //angles[16] = constrain(angles[16], 70, 110);  //servo05_Angle
          //angles[17] = constrain(angles[17], 25, 65);   //servo06_Angle
          //angles[18] = constrain(angles[18], 70, 115);  //servo07_Angle

          
          //Wrist is sum and difference of two angles and z rotation coincident with long axis of forearm
          //wrist_pos[0] = map(((servo05_Angle - 90) - (servo06_Angle - 45)), 40, -40, 100, 724);     // wrist pitch
          //wrist_pos[1] = map(((servo05_Angle - 90) + (servo06_Angle - 45)), -40, 40, 300, 924);     // wrist yaw
          //wrist_pos[2] = map(servo07_Angle, 70, 115, 400, 800);                                  // wrist rotation

          wrist_pos[0] = servo05_Angle;     // wrist pitch
          wrist_pos[1] = servo06_Angle;     // wrist yaw
          wrist_pos[2] = map(servo07_Angle, 70, 115, 400, 800);                                  // wrist rotation

          //Serial.print("Wrist: ");
          sc.WritePos(1, wrist_pos[0], 200, 400);               // bottom outer
          sc.WritePos(2, wrist_pos[1], 200, 400);               // top outer
          //sc.WritePos(3, wrist_pos[2], 50, 200);               // 3 wrist rotation

          //Serial.println("5:"+String(servo05_Angle) + ", [0]:"+String(wrist_pos[0])+",       6:"+String(servo06_Angle) + ", [1]:"+String(wrist_pos[1]));
        }
      }
}

