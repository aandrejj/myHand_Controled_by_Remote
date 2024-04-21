#include "Arduino.h"
#include "MyLcd_SplashScreen.h"
#include "FormData.h"

	MyLcd_SplashScreen::MyLcd_SplashScreen()
  {
		
	}

	void MyLcd_SplashScreen::begin()
  {
		
	}

	FormData MyLcd_SplashScreen::showForm()
  {
    String lcdString0= " Dexterous Hand v1.0";
	  String lcdString1= " www.andrejRobotika ";
	  String lcdString2= "    .blogspot.sk    ";
	  String lcdString3= "                    ";

		FormData  formData(lcdString0, lcdString1, lcdString2, lcdString3);

    return formData;
	}
