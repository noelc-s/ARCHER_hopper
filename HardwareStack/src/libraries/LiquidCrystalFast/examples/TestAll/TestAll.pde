#include <LiquidCrystalFast.h>
// Try to exercise everything in LiquidCrystal
uint8_t nRows = 2;      //number of rows on LCD
uint8_t nColumns = 16;  //number of columns

// initialize the library with the numbers of the interface pins
LiquidCrystalFast lcd(12, 10, 11, 5, 4, 3, 2);
         // LCD pins: RS  RW  EN  D4 D5 D6 D7

void setup(void) {
	randomSeed(analogRead(0));
	lcd.begin(nColumns,nRows);
}

void loop(void) {
	lcd.clear();
	lcd.setCursor((nColumns>>1)-2,0);
	lcd.print( (int)nColumns);
	lcd.print(",");
	lcd.print( (int)nRows);
	delay(1000);

	//mark the corners
	lcd.setCursor(0,0);
	lcd.print('1');
	lcd.setCursor(nColumns-1,nRows-1);
	lcd.print('4');
	lcd.setCursor(nColumns-1,0);
	lcd.print('2');
	lcd.setCursor(0,nRows-1);
	lcd.print('3');
	lcd.setCursor((nColumns>>1)-3,0);
	//  lcd.print("corners ");
	delay(1000);
	lcd.setCursor((nColumns>>1)-3,0);
	lcd.print("  scroll ");
	uint8_t rand=random(150);
	lcd.setCursor((nColumns>>1)-3,1);
	lcd.print((int)rand);
	int i = 0;
	while (i < rand) {
		lcd.scrollDisplayLeft(); 
		i++;
		delay(100);
	}
	// now we repeat the corner code to be sure we fixed the behavior of setCursor after scroll
	delay(1000);
	//mark the corners
	lcd.setCursor(0,0);
	lcd.print('a');
	lcd.setCursor(nColumns-1,nRows-1);
	lcd.print('d');
	lcd.setCursor(nColumns-1,0);
	lcd.print('b');
	lcd.setCursor(0,nRows-1);
	lcd.print('c');
	lcd.setCursor((nColumns>>1)-3,0);
	lcd.print("corners ");
	delay(1000);
	
	lcd.clear();
	lcd.setCursor(0,0);
	lcd.print('1');
	lcd.setCursor(nColumns-1,nRows-1);
	lcd.print('4');
	lcd.setCursor(nColumns-1,0);
	lcd.print('3');
	lcd.setCursor(0,nRows-1);
	lcd.print('2');
	lcd.setCursor((nColumns>>1)-3,0);
	lcd.print("corners");
	delay(1000);
	lcd.setCursor((nColumns>>1)-3,1);
	lcd.print("  scroll ");
	rand = random(150);
	lcd.setCursor((nColumns>>1)-3,2);
	lcd.print((int) rand);
	i = 0;
	while (i < rand) {
		lcd.scrollDisplayRight(); 
		i++;
		delay(100);
	}
	// now we repeat the corner code to be sure we fixed the behavior of setCursor after scroll
	delay(1000);
	//mark the corners  -- all of these corner tests are good ways to find off by one errors!
	
	lcd.setCursor(nColumns-1,nRows-1);
	lcd.print('d');
	lcd.setCursor(nColumns-1,0);
	lcd.print('b');
	lcd.setCursor(0,nRows-1);
	lcd.print('c');
        lcd.setCursor(0,0);
	lcd.print('a');
	lcd.setCursor((nColumns>>1)-3,0);
	lcd.print("corners ");
	delay(1000);
	
	//Cursor  on/off
	lcd.clear();
	lcd.setCursor (0,0);
	lcd.print("Cursor off");
	lcd.noCursor();
	delay(1000);
	// Turn on the cursor:
	lcd.setCursor (0,0);
	lcd.print("Cursor on ");
	lcd.cursor();
	delay(1000);
	lcd.setCursor (0,nRows-1);
	lcd.print("Cursor off");
	lcd.noCursor();
	delay(1000);
	// Turn on the cursor:
	lcd.setCursor (0,nRows-1);
	lcd.print("Cursor on ");
	lcd.cursor();
	delay(1000);
	//==enumerate lines on the display 
	lcd.clear();
	i = 0;
	while (i < nRows) {  
		lcd.setCursor(nColumns-8,i);
		lcd.print("ROW ");
		lcd.print(i+1);
		i++;
	}
	delay(1000);
	//=====try println
	lcd.clear();
        lcd.setCursor(0,0);
	i = 0;
	while (i < nRows) {  
		lcd.print("Println:# ");
		lcd.println(i+1);
		i++;
	}
	delay(1000);
	
	//=====Line wrap demo:
	lcd.clear();
	lcd.setCursor(nColumns-4,0);
	lcd.print("LINEWRAP01234567890");
	delay(200);
	if (nRows>=2) {
		lcd.setCursor(nColumns-4,1);
		lcd.print("linewrap01234567890");
		delay(1000);
		
		if (nRows >=4) {
			lcd.setCursor(nColumns-4,3);
			lcd.print("linewrap01234567890");
			delay(200); 
		}
	}
	delay(1000);
	i = 0;
	while (i < 11) {
		lcd.scrollDisplayLeft(); 
		i++;
		delay(100);
	}
	
	delay(1000);
	lcd.home();
	int length = nRows * nColumns;
	lcd.setCursor(0,0);
	char text[]="ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ";
	text[length] = '\0';
	
    lcd.print(text);  //here we are printing the entire screen with one long string  lets you be sure that linewrap happens exactly as it should
    delay(2000);
   uint8_t a = 0;
  if ((nColumns == 8) && (nRows == 2) ) a=1;  //This is a trick to make a 'crazy 8' 16x1 LCD work reasonably in a right to left language; I suppose it could be added to the library routine
                                              // as a conditional but  that seems like too much support. what happens is that we write the 2nd line first; when that fills with (8) characters
                                              // it wraps onto the first line. This means that lcd.print("abcdefghijklmno"); will print from right to left as you desire.
    lcd.clear();
    lcd.rightToLeft();
    lcd.setCursor(nColumns-1,a); //now we print it in right to left mode to test the same thing
    lcd.print (text);
    delay(3000);
    lcd.leftToRight();
    
    long startTime=millis();    //let's try to benchmark how fast we can go; this will give us an idea about speed of the various interfaces 4/8 bit and checking busy flag or not:
    uint8_t repetitions = 20;
    char blanks[]="                                                                                                                                                                                                                                                                                                                                                 ";
    blanks[length] = '\0';
    while (repetitions--) {  //fill every screen pixel with text, then fill every pixel with blanks and repeat.
      lcd.setCursor(0,0);
      lcd.print(text);
      lcd.setCursor(0,0);
      lcd.print(blanks);
    }
    long endTime = millis();
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Benchmark took ");
    lcd.setCursor(0,1);
    lcd.print(endTime - startTime);
    lcd.print(" millisecs.");
    delay(5000);
    
    
	//======setCursor===
	// loop from ASCII 'a' to ASCII 'z':
	lcd.home();        
	char thisLetter = 'a';
	// loop over the rows:
	for (int thisRow = 0; thisRow < nRows; thisRow++) {
		// loop over the columns:
		for (int thisCol = 0; thisCol < nColumns; thisCol++) {
			// set the cursor position:
			lcd.setCursor(thisCol,thisRow);
			// print the letter:
			lcd.print(thisLetter);
			thisLetter++;
			if (thisLetter > 'z') thisLetter = 'a';
			delay(100);
		}
	}
	
	
	//========Autoscroll:  -- my arch nemesis !
	lcd.clear();
	// set the cursor to (0,0):
	lcd.setCursor(0, 0);
	// print from 0 to 9:
	lcd.print("Autoscroll");
	for (char thisChar = '1'; thisChar < '9'; thisChar++) {
		lcd.print(thisChar);
		delay(100);
	}
	
	// set the cursor to (nColumns,1):
	lcd.setCursor(0,1);
        lcd.print("Autoscroll");
	// set the display to automatically scroll:
	lcd.autoscroll();
	
	
	// print from 0 to 9:
	for (int thisChar = 0; thisChar < 10; thisChar++) {
		lcd.print(thisChar);
		delay(100);
	}
	// turn off automatic scrolling
	lcd.noAutoscroll();
	if (nRows>2) {
		//========Autoscroll:  -- my arch nemesis !

		// set the cursor to (0,0):
		lcd.setCursor(0, nRows-2);
		// print from 0 to 9:
		lcd.print("Autoscroll");
		for (char thisChar = '1'; thisChar < '9'; thisChar++) {
			lcd.print(thisChar);
			delay(100);
		}
	
		// set the cursor to (nColumns,1):
		lcd.setCursor(0,nRows-1);
        	lcd.print("Autoscroll");
		// set the display to automatically scroll:
		lcd.autoscroll();
	
	
		// print from 0 to 9:
		for (int thisChar = 0; thisChar < 10; thisChar++) {
			lcd.print(thisChar);
			delay(200);
		}
		// turn off automatic scrolling
		lcd.noAutoscroll();
	}	
	//====== Turn on the blinking cursor:
	lcd.clear();
	lcd.setCursor(0,nRows-1);
	lcd.print("Blinking Cursor");
	lcd.blink();
	delay(1000);
	// Turn off the blinking cursor:
	lcd.noBlink();
	delay(1000);
	lcd.setCursor(0,0);
	lcd.print("Blinking Cursor");
	lcd.blink();
	delay(1000);
	// Turn off the blinking cursor:
	lcd.noBlink();
	delay(1000);
	
	//=====Turn display on and off:
	lcd.clear();
	lcd.setCursor(0,0);
	lcd.print("turn display off");
	delay(1000);
	// Turn off the display:
	lcd.noDisplay();
	delay(2000);
	lcd.setCursor(0,0);
	lcd.print("Turn Display ON ");  //sent to the LCD while the display is turned off.
	// Turn on the display:
	lcd.display();
	delay(1000);
	
	lcd.clear();
	lcd.home();
	lcd.setCursor(0,0);
	lcd.print(" Shift under program control ");
	// scroll 27 positions (display length + string length) to the left:
	for (int positionCounter = 0; positionCounter < 27; positionCounter++) {
		// scroll one position left:
		lcd.scrollDisplayLeft(); 
		// wait a bit:
		delay(100);
	}
	
	// scroll 27 positions (display length + string length) to the right:
	for (int positionCounter = 0; positionCounter < 27; positionCounter++) {
		// scroll one position right:
		lcd.scrollDisplayRight(); 
		// wait a bit:
		delay(100);
	}
	delay(1500);
	//======Text direction
	lcd.clear();
	lcd.setCursor(0,0);
	char thisChar ='a';
	for (int i = 0;i!=30; i++) {
		// reverse directions at 'm':
		if (thisChar == 'm') {
			// go right for the next letter
			lcd.rightToLeft(); 
		}
		// reverse again at 's':
		if (thisChar == 's') {
			// go left for the next letter
			lcd.leftToRight(); 
		}
		// reset at 'z':
		if (thisChar > 'z') {
			// go to (0,0):
			lcd.home(); 
			// start again at 0
			thisChar = 'a';
		}
		// print the character
		lcd.print(thisChar);
		
		delay(200);
		//increment the letter:
		thisChar++;
	}
	lcd.clear();
	
	//======define charset
	uint8_t bell[8] = {0x4,0xe,0xe,0xe,0x1f,0x0,0x4};
	uint8_t note[8] = {0x2,0x3,0x2,0xe,0x1e,0xc,0x0};
	uint8_t clock[8] = {0x0,0xe,0x15,0x17,0x11,0xe,0x0};
	uint8_t heart[8] = {0x0,0xa,0x1f,0x1f,0xe,0x4,0x0};
	uint8_t duck[8] = {0x0,0xc,0x1d,0xf,0xf,0x6,0x0};
	uint8_t check[8] = {0x0,0x1,0x3,0x16,0x1c,0x8,0x0};
	uint8_t cross[8] = {0x0,0x1b,0xe,0x4,0xe,0x1b,0x0};
	uint8_t retarrow[8] = { 0x1,0x1,0x5,0x9,0x1f,0x8,0x4};
	
	
	lcd.createChar(0, bell);
	lcd.createChar(1, note);
	lcd.createChar(2, clock);
	lcd.createChar(3, heart);
	lcd.createChar(4, duck);
	lcd.createChar(5, check);
	lcd.createChar(6, cross);
	lcd.createChar(7, retarrow);
	lcd.home();
	
	i = 0;
	lcd.clear();
	while (i<nRows) {
		lcd.setCursor(0,i);
		lcd.print("user:");
		for (int j=0; j<7; j++) {
			lcd.write(j);
		}
		
		i++;
	}
	delay(2000);
	
	
}

