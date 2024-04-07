#ifndef MyLcd_SplashScreen_h
#define MyLcd_SplashScreen_h

#include "Arduino.h"
#include "FormData.h"

class MyLcd_SplashScreen {
	public:
	MyLcd_SplashScreen();

	void begin();

	FormData showForm();

};
#endif