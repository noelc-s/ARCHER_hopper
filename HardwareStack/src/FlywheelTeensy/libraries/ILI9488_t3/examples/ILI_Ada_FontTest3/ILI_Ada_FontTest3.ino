//-------------------------------------------------------------------
//
// This test program illustrates a one method of using ILI9341 fonts 
// and Adafruit fonts with ILI9488 display.
// 
// this sketch is in the public domain. 
//
// This sketch depends on the fonts that are contained in the library
//     https://github.com/mjs513/ILI9341_fonts
//-------------------------------------------------------------------
#include <Adafruit_GFX.h>

#include <SPI.h>
#include "ILI9488_t3.h"

#include "font_Arial.h"
#include "font_ArialBold.h"
#include "font_ComicSansMS.h"
#include "font_OpenSans.h"
#include "font_DroidSans.h"
#include "font_Michroma.h"
#include "font_Crystal.h"
#include "font_ChanceryItalic.h"

#define ILI9488_CS 10
#define ILI9488_DC 9
#define ILI9488_RST 8
ILI9488_t3 tft = ILI9488_t3(&SPI, ILI9488_CS, ILI9488_DC, ILI9488_RST);
uint8_t test_screen_rotation = 0;


void setup() {
  Serial.begin(38400);
  long unsigned debug_start = millis ();
  while (!Serial && ((millis () - debug_start) <= 5000)) ;
  Serial.println("Setup");
  tft.begin();

  tft.setRotation(4);
  tft.fillScreen(ILI9488_BLACK);

  tft.setTextColor(ILI9488_WHITE);
  tft.setFont(Arial_12);
  tft.println("Arial_12");
  displayStuff();

  tft.setTextColor(ILI9488_YELLOW);
  tft.setFont(Arial_12_Bold);
  tft.println("ArialBold 12");
  displayStuff();

  tft.setTextColor(ILI9488_GREEN);
  tft.setFont(ComicSansMS_12);
  tft.println("ComicSansMS 12");
  displayStuff(); 

  nextPage();
  
  tft.setTextColor(ILI9488_WHITE);
  tft.setFont(DroidSans_12);
  tft.println("DroidSans_12");
  displayStuff();

  tft.setTextColor(ILI9488_YELLOW);
  tft.setFont(Michroma_12);
  tft.println("Michroma_12");
  displayStuff();

  tft.setTextColor(ILI9488_BLACK, ILI9488_YELLOW);
  tft.setFont(Crystal_18_Italic);
  tft.println("CRYSTAL_18");
  displayStuff();

  nextPage();

  tft.setTextColor(ILI9488_GREEN);
  tft.setFont(Chancery_18_Italic);
  tft.println("Chancery_18_Italic");
  displayStuff();

  //anti-alias font OpenSans
  tft.setTextColor(ILI9488_RED, ILI9488_YELLOW);
  tft.setFont(OpenSans18);
  tft.println("OpenSans 18");
  displayStuff(); 
  
  Serial.println("Basic Font Display Complete");
  Serial.println("Loop test for alt colors + font");
}

void loop()
{
  Serial.printf("\nRotation: %d\n", test_screen_rotation);
  tft.setRotation(test_screen_rotation);
  test_screen_rotation = (test_screen_rotation + 1) & 0x3;
  
  nextPage();

  tft.setTextColor(ILI9488_WHITE);
  tft.setFont(Arial_12);
  tft.println("Arial_12");
  displayStuff1();

  tft.setTextColor(ILI9488_YELLOW);
  tft.setFont(Arial_12_Bold);
  tft.println("ArialBold 14");
  displayStuff1();

  nextPage();

  tft.setTextColor(ILI9488_GREEN);
  tft.setFont(ComicSansMS_12);
  tft.println("ComicSansMS 14");
  displayStuff1(); 

  tft.setTextColor(ILI9488_WHITE);
  tft.setFont(DroidSans_12);
  tft.println("DroidSans_12");
  displayStuff1();

  nextPage();

  tft.setTextColor(ILI9488_YELLOW);
  tft.setFont(Michroma_12);
  tft.println("Michroma_12");
  displayStuff1();

  nextPage();
  
  tft.setTextColor(ILI9488_BLACK, ILI9488_YELLOW);
  tft.setFont(Crystal_18_Italic);
  tft.println("CRYSTAL_18");
  displayStuff1();

  tft.setTextColor(ILI9488_GREEN);
  tft.setFont(Chancery_18_Italic);
  tft.println("Chancery_18_Italic");
  displayStuff1();
  
  nextPage();

  //anti-alias font OpenSans
  tft.setTextColor(ILI9488_RED, ILI9488_YELLOW);
  tft.setFont(OpenSans18);
  tft.println("OpenSans 18");
  displayStuff1(); 

  nextPage();
}

uint32_t displayStuff()
{
  elapsedMillis elapsed_time = 0;
  tft.println("ABCDEFGHIJKLMNOPQRSTUVWXYZ");
  tft.println("abcdefghijklmnopqrstuvwxyz");
  tft.println("0123456789");
  tft.println("!@#$%^ &*()-");
  tft.println(); tft.println();
  return (uint32_t) elapsed_time;
}

uint32_t displayStuff1()
{
  elapsedMillis elapsed_time = 0;
  tft.println("ABCDEFGHIJKLMNOPQRSTUVWXYZ");
  tft.println("abcdefghijklmnopqrstuvwxyz");
  tft.println("0123456789");
  tft.println("!@#$%^ &*()-");
  static const char alternating_text[] = "AbCdEfGhIjKlMnOpQrStUvWxYz\raBcDeFgHiJkLmNoPqRsTuVwXyZ";
 
  for (uint8_t i = 0; i < sizeof(alternating_text); i++) {
    if (i & 1) tft.setTextColor(ILI9488_WHITE, ILI9488_RED);
    else tft.setTextColor(ILI9488_YELLOW, ILI9488_BLUE);
    tft.write(alternating_text[i]);
  }
  tft.println(); tft.println();
  return (uint32_t) elapsed_time;
}

void nextPage()
{
  Serial.println("Press anykey to continue");
  while (Serial.read() == -1) ;
  while (Serial.read() != -1) ;

  tft.fillScreen(ILI9488_BLACK);
  tft.setCursor(0, 0);
}