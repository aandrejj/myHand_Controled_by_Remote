#ifndef FormData_h
#define FormData_h

#include "Arduino.h"


class FormData {
	public:
	String lcdString0;
	String lcdString1;
	String lcdString2;
	String lcdString3;

	FormData(String LcdString0, String LcdString1, String LcdString2, String LcdString3);
	
};
#endif
