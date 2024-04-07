#include "Arduino.h"
#include "FormData.h"


FormData::FormData(String LcdString0, String LcdString1, String LcdString2, String LcdString3) {
	FormData::lcdString0 = LcdString0;
	FormData::lcdString1 = LcdString1;
	FormData::lcdString2 = LcdString2;
	FormData::lcdString3 = LcdString3;
}
