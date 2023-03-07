/*
Demonstrate FT5206 gesture ID's
Works in all orientations

one finger swipe -> up, down ,left, right
two finger pinch -> zoom in, zoom out
three finger tap -> change screen rotation
*/

#include <SPI.h>
#include <RA8875.h>

/*
Teensy3.x and Arduino's
You are using 4 wire SPI here, so:
 MOSI:  11//Teensy3.x
 MISO:  12//Teensy3.x
 SCK:   13//Teensy3.x
 the rest of pin below:
*/

#define RA8875_INT        2 //any pin
#define RA8875_CS         10 //any digital pin
#define RA8875_RESET      9 //any pin or nothing!

#define MAXTOUCHLIMIT     3 //1...5

RA8875 tft = RA8875(RA8875_CS, RA8875_RESET); //Teensy3/arduino's


void setup(){
  //  begin display: Choose from: RA8875_480x272, RA8875_800x480, RA8875_800x480ALT, Adafruit_480x272, Adafruit_800x480
  tft.begin(RA8875_800x480);
  tft.setRotation(0);
  #if defined(USE_FT5206_TOUCH)
  tft.useCapINT(RA8875_INT);
  tft.setTouchLimit(MAXTOUCHLIMIT);
  tft.enableCapISR(true);
  tft.setTextColor(RA8875_WHITE,RA8875_BLACK);
  printHeader();
  #else
  tft.print("you should open RA8875UserSettings.h file and uncomment USE_FT5206_TOUCH!");
  #endif
}

void loop(){
  #if defined(USE_FT5206_TOUCH)
  static uint8_t rot = 0;
  static uint8_t prev_touches = 0;
  if (tft.touched()){
    tft.updateTS();
    if(tft.getTouches() == 3 && prev_touches < 3){
      if(++rot > 3) rot = 0;
      tft.setRotation(rot);
      printHeader();
    }else{
      uint8_t gesture = tft.getGesture();
      tft.setFontScale(1);
      switch (gesture) {
        case 0x10: //up
        tft.setCursor(CENTER,CENTER);
        tft.print("UP      ");
        break;
        case 0x14: //left
        tft.setCursor(CENTER,CENTER);
        tft.print("LEFT    ");
        break;
        case 0x18: //down
        tft.setCursor(CENTER,CENTER);
        tft.print("DOWN    ");
        break;
        case 0x1C: //right
        tft.setCursor(CENTER,CENTER);
        tft.print("RIGHT   ");
        break;
        case 0x48: //zoom in
        tft.setCursor(CENTER,CENTER);
        tft.print("ZOOM IN ");
        break;
        case 0x49: //zoom out
        tft.setCursor(CENTER,CENTER);
        tft.print("ZOOM OUT");
        break;
      }
    }
    prev_touches = tft.getTouches();
  }
  #endif
}
void printHeader(){
  tft.fillWindow(RA8875_BLACK);
  tft.setFontScale(0);
  tft.setCursor(0, 0);
  tft.print("Swipe or Pinch.");
  tft.setCursor(0, 25);
  tft.print("Tap 3 fingers to rotate.");
}
