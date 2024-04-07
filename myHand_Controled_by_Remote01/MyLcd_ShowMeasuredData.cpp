#include "Arduino.h"
#include "MyLcd_ShowMeasuredData.h"
#include "FormData.h"

	MyLcd_ShowMeasuredData::MyLcd_ShowMeasuredData()
    {
		
	}

	void MyLcd_ShowMeasuredData::begin()
  {
		
	}

	FormData MyLcd_ShowMeasuredData::showForm(int leftJoystick_X, int rightJoystick_X, int leftJoystick_Y, int rightJoystick_Y, String btnsString, String lcdString3)
  {
    String lcdString0= "LX:"+String(leftJoystick_X)+", RX:"+String(rightJoystick_X)+"  ";
	String lcdString1= "LY:"+String(leftJoystick_Y)+",  RY:"+String(rightJoystick_Y)+"  ";
	String lcdString2= btnsString;
	//String lcdString3= "                    ";

	FormData  formData(lcdString0, lcdString1, lcdString2, lcdString3);

    return formData;
	}

	FormData MyLcd_ShowMeasuredData::showForm2(String leftJoystick_X, String rightJoystick_X, String leftJoystick_Y, String rightJoystick_Y, String btnsString, String lcdString3)
  {
    String lcdString0= "LX:"+leftJoystick_X+",RX"+rightJoystick_X+" ";
	  String lcdString1= "LY:"+leftJoystick_Y+",RY"+rightJoystick_Y+" ";
	  String lcdString2= btnsString;
	//String lcdString3= "                    ";

	FormData  formData(lcdString0, lcdString1, lcdString2, lcdString3);

    return formData;
	}
