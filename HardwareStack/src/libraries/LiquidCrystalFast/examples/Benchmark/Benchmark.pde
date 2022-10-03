#include <LiquidCrystalFast.h>

// initialize the library with the numbers of the interface pins
LiquidCrystalFast lcd(12, 10, 11, 5, 4, 3, 2);
         // LCD pins: RS  RW  EN  D4 D5 D6 D7

// to see the speed of the original library, comment out the
// LiquidCrystalFast line above, and uncomment these 2 lines.
//#include <LiquidCrystal.h>
//LiquidCrystal lcd(12, 10, 11, 5, 4, 3, 2);

const int nRows = 4;      //number of rows on LCD
const int nColumns = 16;  //number of columns

const int length = nRows * nColumns;
char text[length+1];
char blanks[length+1];

void setup(void) {
	lcd.begin(nColumns,nRows);
	lcd.setCursor(0,0);
	char c = 'A';
	for (int i=0; i<length; i++) {
		text[i] = c++;
		blanks[i] = ' ';
		if (c > 'Z') c = 'A';
	}
	text[length] = 0;
	blanks[length] = 0;
	unsigned long startTime=millis();
	byte repetitions = 20;
	while (repetitions--) {
		lcd.setCursor(0,0);  // fill every screen pixel with text
		lcd.print(text);
		lcd.setCursor(0,0);  // then fill every pixel with blanks and repeat
		lcd.print(blanks);
	}
	unsigned long endTime = millis();
	lcd.clear();
	lcd.setCursor(0,0);
	lcd.print("Benchmark ");
	lcd.print(nColumns, DEC);
	lcd.write('x');
	lcd.print(nRows, DEC);
	lcd.setCursor(0,1);
	lcd.print(endTime - startTime);
	lcd.print(" millisecs.");
}

void loop() {
}

