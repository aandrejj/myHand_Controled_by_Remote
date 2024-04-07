#ifndef MyLcd_h
#define MyLcd_h

#include "Arduino.h"
#include "LiquidCrystal_I2C.h"
#include "FormData.h"

//#define I2C_ADDR 0x27  //just a few 


class MyLcd {
	public:
  String lcdString0;
  String lcdString1;
  String lcdString2;
  String lcdStringMain;

  //byte showForm;

	MyLcd();

  void begin(LiquidCrystal_I2C lcd);
  //void setShownFromTo(byte ShowForm);
  void scroll_text_on_display();
  void print(char c);
  void print(String text);
  void println(String text);
  void clear();
  void showSplashScreen();
  void showMeasuredDateScreen(int leftJoystick_X, int rightJoystick_X, int leftJoystick_Y, int rightJoystick_Y, String btnsString, String lcdString3);
  void showMeasuredDateScreen2(int LJoyX, int LJoyY, int RJoyX, int RJoyY, int lLX, int lLY, int lRX, int lRY, String btnsString, String lcdString3);
  void formShow(FormData formData);
  void ShowNewForm(String lcdString0, String lcdString1, String lcdString2, String lcdString3);
  void formShow(String lcdString0, String lcdString1, String lcdString2, String lcdString3);

};
#endif