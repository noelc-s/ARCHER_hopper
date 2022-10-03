//-------------------------------------------------------------------
//
// This test program illustrates a simple use of ILI9341 fonts with the
// ILI9488 display.
// 
// this sketch is in the public domain. 
//
// This sketch depends on the fonts that are contained in the library
//     https://github.com/mjs513/ILI9341_fonts
//-------------------------------------------------------------------


#include <Adafruit_GFX.h>

#include <SPI.h>
#include <ILI9488_t3.h>

#include <ILI9488_t3_font_ComicSansMS.h>
#include "Fonts/FreeSansOblique12pt7b.h"

#define TFT_RST 8
#define TFT_DC  9  // only CS pin 
#define TFT_CS  10   // using standard pin
ILI9488_t3 tft = ILI9488_t3(&SPI, TFT_CS, TFT_DC, TFT_RST);

void setup() {
  Serial.begin(38400);
  long unsigned debug_start = millis ();
  while (!Serial && ((millis () - debug_start) <= 5000)) ;
  Serial.println("Setup");
  tft.begin();

  tft.setRotation(1);
  tft.fillScreen(ILI9488_BLACK);

  tft.setTextSize(2);
  tft.setTextColor(ILI9488_WHITE);
  tft.println("Test of the default font");
  tft.println();
  tft.setTextSize(1);

  tft.setTextColor(ILI9488_WHITE, ILI9488_BLUE);
  tft.setFont(ComicSansMS_12);
  tft.println("Opaque ILI font ComicSansMS_12");
  tft.println();

  tft.setTextColor(ILI9488_GREEN);
  tft.println("Transparent ILIComicSansMS_12");
  tft.println();

  tft.setTextColor(ILI9488_WHITE, ILI9488_RED);
  tft.setFont(&FreeSansOblique12pt7b);
  tft.println("Opaque GFX FreeSansOblique12pt");
  int cursor_x = tft.getCursorX();
  int cursor_y = tft.getCursorY();
  tft.fillRect(cursor_x, cursor_y, tft.width(), 20, ILI9488_YELLOW);
  tft.setTextColor(ILI9488_BLUE);
  tft.println("Transparent GFX");

  tft.setFont(); tft.setTextSize(2);
  tft.setTextColor(ILI9488_GREEN);
  tft.setTextSize(1);
  tft.println("This is default font:");
  //tft.setFontSpacing(1);//now give 5 pix extra spacing between chars
  tft.println("ABCDEF 1 2 3 4 5 6 7");

}

void loop()
{  }