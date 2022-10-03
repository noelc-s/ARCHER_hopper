/*
example of drawLineAngle
*/

#include <SPI.h>
#include <RA8875.h>


#define RA8875_CS 10

#define RA8875_RESET 9//any pin or 255 to disable it!


RA8875 tft = RA8875(RA8875_CS, RA8875_RESET);

uint16_t x1, x2, x3, y;

void setup()
{

  //  begin display: Choose from: RA8875_480x272, RA8875_800x480, RA8875_800x480ALT, Adafruit_480x272, Adafruit_800x480
  tft.begin(RA8875_800x480);

  x1 = tft.width() / 2;
  x2 = tft.width() / 2 + 200;
  x3 = tft.width() / 2 - 200;
  y = tft.height() / 2;
  
  tft.drawCircle(x1, y, 100, 0xFFFF);//draw round gauge

  tft.drawCircle(x2, y, 100, 0xFFFF);
  tft.drawCircle(x2, y, 33, 0xFFFF);

  tft.drawCircle(x3, y, 100, 0xFFFF);
  tft.drawCircle(x3, y, 50, 0xFFFF);
}

void loop()
{
  for (uint16_t i = 0; i < 360; i++) {
    //erase previous
    tft.drawLineAngle(x1, y, -i + 1, 100, 0x0000, 0);//erase previous needle
    tft.drawLineAngle(x2, y, i*2 - 2, 34, 66, 0x0000, 180);
    tft.drawLineAngle(x3, y, i - 1, 75, 1, 0x0000, 180);
    tft.drawLineAngle(x3, y, i - 1, 75, 1, 0x0000, 60);
    tft.drawLineAngle(x3, y, i - 1, 75, 1, 0x0000, 300);
    //draw
    tft.drawLineAngle(x1, y, -i, 100, 0xFFFF, 0);//draw needle from center, if offset is not passed, it will default to -90.0
    tft.drawLineAngle(x2, y, i*2, 34, 66, 0xFFFF, 180);//draw needle some distance from center
    tft.drawLineAngle(x3, y, i, 75, 1, 0xFFFF, 180);//draw pixel some distance from center
    tft.drawLineAngle(x3, y, i, 75, 1, 0xFFFF, 60);
    tft.drawLineAngle(x3, y, i, 75, 1, 0xFFFF, 300);
    delay(10);
  }
}
