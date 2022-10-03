/* Testing Resistive Touch Screen
*/

#include <SPI.h>
#include <RA8875.h>

/*
Teensy3.x
You are using 4 wire SPI here, so:
 MOSI:  11//Teensy3.x
 MISO:  12//Teensy3.x
 SCK:   13//Teensy3.x
 the rest of pin below:
 */

#define RA8875_INT 3 //any pin that have an INT
#define RA8875_CS 10 // Can be any digital pin
#define RA8875_RESET 9 //any pin or nothing!

#if !defined(USE_RA8875_TOUCH) || defined(_AVOID_TOUCHSCREEN)
#error "you need to enable resistive touchscreen by uncommenting USE_RA8875_TOUCH in settings!"
#endif

static uint16_t tx, ty;//used as temp
uint16_t  txPrev = 0xffff;
uint16_t  tyPrev = 0xffff;
uint16_t  minX = 0xffff;
uint16_t  maxX = 0;
uint16_t  minY = 0xffff;
uint16_t  maxY = 0;

uint16_t rotation_colors[4] = {RA8875_RED, RA8875_GREEN, RA8875_BLUE, RA8875_YELLOW};

RA8875 tft = RA8875(RA8875_CS, RA8875_RESET);//Teensy3/arduino's

void setup() {
  Serial.begin(38400);
  long unsigned debug_start = millis ();
  while (!Serial && ((millis () - debug_start) <= 5000)) ;
  //  begin display: Choose from: RA8875_480x272, RA8875_800x480, RA8875_800x480ALT, Adafruit_480x272, Adafruit_800x480
  tft.begin(RA8875_480x272, 16, 10000000);//initialize RA8875
  /* Adafruit_480x272 Adafruit_800x480 RA8875_480x272 RA8875_800x480 */
  tft.useINT(RA8875_INT);//We use generic int helper for Internal Resistive Touch
  tft.touchBegin();//enable touch support for RA8875
  // Optionally you can set the calibration of your display manually
  //    tft.setTouchCalibrationData(0,0,0,0);
  if (!tft.touchCalibrated()) {//already calibrated?
    Serial.println("Maybe better you calibrate first!");
    Serial.println("Clearing it out");
  }
  //this enable an ISR on CPU and RA8875 INT
  tft.enableISR(true);
  //You can avoid ISR by simple ignore the line above
  //it will use the slower digitalRead(pin) alternative internally
  tft.fillWindow(rotation_colors[0]);

}

void loop() {
  if (tft.touched()) {
    //if you are here means that you touched screen
    tft.touchReadPixel(&tx,&ty);//read directly in pixel
    if ((tx != txPrev) || (ty != tyPrev)) {
      tft.fillCircle(tx,ty,5,0xFFFF);
      Serial.printf("TX: %u TY: %u\n", tx, ty);     
      if (tx < minX) minX = tx;
      if (tx > maxX) maxX = tx;
      if (ty < minY) minY = ty;
      if (ty > minY) maxY = ty;
    }
  }
  if (Serial.available()) {
    Serial.printf("X Range: %u %u Y Range: %u %u\n", minX, maxX, minY, maxY);
    uint8_t rot = Serial.read(); 
    if ((rot >= '0') && (rot <= '3')) {
      rot -= '0';
      tft.setRotation(rot);
      tft.fillWindow(0);
      Serial.printf("\n\n*** Set Rotation(%d) ***\n", rot);
      tft.fillWindow(rotation_colors[rot]);
      minX = 0xffff;
      maxX = 0;
      minY = 0xffff;
      maxY = 0;
    }
    while (Serial.read() != -1) ; // clear out rest of data
  }
}
