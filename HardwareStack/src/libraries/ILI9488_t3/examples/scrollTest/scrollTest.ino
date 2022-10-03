/***************************************************
  This is our GFX example for the Adafruit ILI9488 Breakout and Shield
  ----> http://www.adafruit.com/products/1651

  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/


#include <SPI.h>
#include <ILI9488_t3.h>
#include <ILI9488_t3_font_ComicSansMS.h>

//#define TEENSY64

// For the Adafruit shield, these are the default.
#if defined(__MK66FX1M0__) && !defined(TEENSY64)
#define TFT_RST 8
#define TFT_DC 9
#define TFT_CS 10
ILI9488_t3 tft = ILI9488_t3(&SPI, TFT_CS, TFT_DC, TFT_RST);
#elif defined(__IMXRT1052__) || defined(__IMXRT1062__)
// On Teensy 4 beta with Paul's breakout out:
// Using pins (MOSI, MISO, SCK which are labeled on Audio board breakout location
// which are not in the Normal processor positions
// Also DC=10(CS), CS=9(BCLK) and RST 23(MCLK)
#define TFT_RST 23
#define TFT_DC 9
#define TFT_CS 10
ILI9488_t3 tft = ILI9488_t3(&SPI, TFT_CS, TFT_DC, TFT_RST);
#elif defined(TEENSY64)
#define TFT_RST 255
#define TFT_DC 20
#define TFT_CS 21
#define TFT_SCK 14
#define TFT_MISO 39
#define TFT_MOSI 28
ILI9488_t3 tft = ILI9488_t3(&SPI, TFT_CS, TFT_DC, TFT_RST, TFT_MOSI, TFT_SCK, TFT_MISO);
#else
#error "This example App will only work with Teensy 3.6 or Teensy 4."
#endif
// If using the breakout, change pins as desired
//Adafruit_ILI9488 tft = Adafruit_ILI9488(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST, TFT_MISO);

void setup() {

  Serial.begin(9600);
 
  tft.begin();

  tft.setRotation(3);

  tft.fillScreen(ILI9488_BLACK);
  while (!Serial) ; 
  tft.setTextColor(ILI9488_WHITE);  tft.setTextSize(4);
  tft.enableScroll();
  tft.setScrollTextArea(0,0,120,240);
  tft.setScrollBackgroundColor(ILI9488_GREEN);

  tft.setCursor(180, 100);

  tft.setFont(ComicSansMS_12);
  tft.print("Fixed text");

  tft.setCursor(0, 0);

  tft.setTextColor(ILI9488_BLACK); 

  for(int i=0;i<20;i++){
    tft.print("  this is line ");
    tft.println(i);
    delay(500);
  }

  tft.fillScreen(ILI9488_BLACK);
  tft.setScrollTextArea(40,50,120,120);
  tft.setScrollBackgroundColor(ILI9488_GREEN);
  tft.setFont(ComicSansMS_10);

  tft.setTextSize(1);
  tft.setCursor(40, 50);

  for(int i=0;i<20;i++){
    tft.print("  this is line ");
    tft.println(i);
    delay(500);
  }


}



void loop(void) {


}
