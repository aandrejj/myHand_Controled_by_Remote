#include "Arduino.h"
#include "MyLcd.h"

#include "MyLcd_SplashScreen.h"
#include "MyLcd_ShowMeasuredData.h"
#include "LiquidCrystal_I2C.h"
#include "FormData.h";

LiquidCrystal_I2C lcdDev(0x27, 20, 4);
MyLcd_SplashScreen myLcdSplashScreen;
MyLcd_ShowMeasuredData myLcdShowMeasuredData;

MyLcd::MyLcd() {

}

void MyLcd::begin(LiquidCrystal_I2C lcd) {
  lcdDev = lcd;
  lcdDev.begin(20,4);   // Initialize the lcd for 20 chars 4 lines, turn on backlight
  lcdDev.backlight();
  myLcdSplashScreen.begin();
}

/*
void MyLcd::setShownFromTo(byte ShowForm) {
  MyLcd::showForm = ShowForm;
}
*/

void MyLcd::scroll_text_on_display() {
  //Serial.println("scroll_text_on_display: lcdStringMain = '"+String(lcdStringMain)+"'");

  MyLcd::lcdString0 = MyLcd::lcdString1;
  MyLcd::lcdString1 = MyLcd::lcdString2;
  MyLcd::lcdString2 = MyLcd::lcdStringMain;
  MyLcd::lcdStringMain="";
  formShow (MyLcd::lcdString0, MyLcd::lcdString1, MyLcd::lcdString2, MyLcd::lcdStringMain);
}

void MyLcd::print(char c) {
  MyLcd::print(String(c));
}

void MyLcd::print(String text) {
  MyLcd::lcdStringMain = MyLcd::lcdStringMain + text;
  lcdDev.setCursor(0,3);
  lcdDev.print(MyLcd::lcdStringMain);
}

void MyLcd::println(String text) {
  MyLcd::scroll_text_on_display();
  MyLcd::print(text);
}

void MyLcd::clear() {
  MyLcd::lcdString0 = "";
  MyLcd::lcdString1 = "";
  MyLcd::lcdString2 = "";
  MyLcd::lcdStringMain = "";
  lcdDev.clear();
}

void MyLcd::ShowNewForm(String lcdString0, String lcdString1, String lcdString2, String lcdString3) {
  MyLcd::lcdString0 = lcdString0;
  MyLcd::lcdString1 = lcdString1;
  MyLcd::lcdString2 = lcdString2;
  MyLcd::lcdStringMain=lcdString3;

  formShow(lcdString0, lcdString1, lcdString2, lcdString3);
}
void MyLcd::formShow(FormData formData) {
  formShow(formData.lcdString0, formData.lcdString1, formData.lcdString2, formData.lcdString3);
}

void MyLcd::formShow(String lcdString0, String lcdString1, String lcdString2, String lcdString3) {
  if(lcdString0.length()>0) {
    lcdDev.setCursor(0,0);
    lcdDev.print(lcdString0);
  }

  if(lcdString1.length()>0) {
    lcdDev.setCursor(0,1);
    lcdDev.print(lcdString1);
  }

  if(lcdString2.length()>0) {
    lcdDev.setCursor(0,2);
    lcdDev.print(lcdString2);
  }
  
  if(lcdString3.length()>0) {
    lcdDev.setCursor(0,3);
    lcdDev.print(lcdString3);  
  }
}

void MyLcd::showSplashScreen() {
  MyLcd::formShow(myLcdSplashScreen.showForm());
}

void MyLcd::showMeasuredDateScreen(int leftJoystick_X, int rightJoystick_X, int leftJoystick_Y, int rightJoystick_Y, String btnsString, String lcdString3) {
  //Serial.println("showMeasuredDateScreen");
  MyLcd::formShow(myLcdShowMeasuredData.showForm(leftJoystick_X, rightJoystick_X, leftJoystick_Y, rightJoystick_Y, btnsString, lcdString3));
}

void MyLcd::showMeasuredDateScreen2(int LJoyX, int LJoyY, int RJoyX, int RJoyY, int lLX, int lLY, int lRX, int lRY, String btnsString, String lcdString3) {
  //Serial.println("showMeasuredDateScreen");
  String LX_str = String(lLX)+"/"+String(LJoyX);
  String LY_str = String(lLY)+"/"+String(LJoyY);
  String RX_str = String(lRX)+"/"+String(RJoyX);
  String RY_str = String(lRY)+"/"+String(RJoyY);
  FormData formData= myLcdShowMeasuredData.showForm2(LX_str, RX_str, LY_str, RY_str, btnsString, lcdString3);
  MyLcd::formShow(formData);
}
