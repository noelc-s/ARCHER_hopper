/*
Tests string alignment

Normally strings are printed relative to the top left corner but this can be
changed with the setTextDatum() function. The library has #defines for:

TL_DATUM = Top left
TC_DATUM = Top centre
TR_DATUM = Top right
ML_DATUM = Middle left
MC_DATUM = Middle centre
MR_DATUM = Middle right
BL_DATUM = Bottom left
BC_DATUM = Bottom centre
BR_DATUM = Bottom right
*/

#include "ILI9488_t3.h"
#include <SPI.h>
#include <ili9488_t3_font_Arial.h>

#define TFT_RST 8
#define TFT_DC 9
#define TFT_CS 10
ILI9488_t3 tft = ILI9488_t3(&SPI, TFT_CS, TFT_DC, TFT_RST);

unsigned long drawTime = 0;

void setup(void) {
  Serial.begin(115200);
  tft.begin();
  tft.setRotation(3);
  tft.setFont(Arial_18);
  //tft.setTextSize(4);
}

void loop() {

  tft.fillScreen(ILI9488_BLACK);
  
  for(byte datum = 0; datum < 9; datum++) {
    tft.setTextColor(ILI9488_WHITE, ILI9488_BLACK);
    
    tft.setTextDatum(datum);
    
    tft.drawNumber(88,160,60);
    tft.fillCircle(160,120,5,ILI9488_RED);
    
    tft.setTextDatum(datum);
    
    tft.setTextColor(ILI9488_YELLOW);
    tft.drawString("TEENSY 4",5,  160,120);
    delay(1000);
    tft.fillScreen(ILI9488_BLACK);
  }

  tft.setTextDatum(MC_DATUM);
  
  tft.setTextColor(ILI9488_BLACK);
  tft.drawString("X",160,120);
  delay(1000);
  tft.fillScreen(ILI9488_BLACK);
  
  tft.setTextDatum(MC_DATUM);
  
  tft.setTextColor(ILI9488_BLACK);
  tft.drawString("X",160,120);
  delay(1000);
  tft.fillScreen(ILI9488_BLACK);

  tft.setTextColor(ILI9488_WHITE, ILI9488_BLUE);

  tft.setTextDatum(MC_DATUM);

  //Test floating point drawing function
  float test = 67.125;
  tft.drawFloat(test, 4, 160, 180);
  delay(1000);
  tft.fillScreen(ILI9488_BLACK);
  test = -0.555555;
  tft.drawFloat(test, 3, 160, 180);
  delay(1000);
  tft.fillScreen(ILI9488_BLACK);
  test = 0.1;
  tft.drawFloat(test, 4, 160, 180);
  delay(1000);
  tft.fillScreen(ILI9488_BLACK);
  test = 9999999;
  tft.drawFloat(test, 1, 160, 180);
  delay(1000);
  
  tft.fillCircle(160,180,5,ILI9488_YELLOW);
  
  tft.setTextDatum(MC_DATUM);
  
  tft.setTextColor(ILI9488_BLACK);
  tft.drawString("X",160,180);

  delay(4000);
}
