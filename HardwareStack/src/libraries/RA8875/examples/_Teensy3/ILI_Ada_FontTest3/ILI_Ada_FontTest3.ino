#include <Adafruit_GFX.h>

#include <SPI.h>
#include <RA8875.h>

#include "font_Arial.h"
#include "font_ArialBold.h"
#include "font_ComicSansMS.h"
#include "font_OpenSans.h"
#include "font_DroidSans.h"
#include "font_Michroma.h"
#include "font_Crystal.h"
#include "font_ChanceryItalic.h"

#define RA8875_CS 10
#define RA8875_RST 8
RA8875 tft = RA8875(RA8875_CS, RA8875_RST);
uint8_t test_screen_rotation = 0;


void setup() {
  Serial.begin(38400);
  long unsigned debug_start = millis ();
  while (!Serial && ((millis () - debug_start) <= 5000)) ;
  Serial.println("Setup");
  //  begin display: Choose from: RA8875_480x272, RA8875_800x480, RA8875_800x480ALT, Adafruit_480x272, Adafruit_800x480
  tft.begin(Adafruit_800x480);

  tft.setRotation(4);
  tft.fillWindow(RA8875_BLACK);

  tft.setTextColor(RA8875_WHITE);
  tft.setFont(Arial_14);
  tft.println("Arial_14");
  displayStuff();

  tft.setTextColor(RA8875_YELLOW);
  tft.setFont(Arial_14_Bold);
  tft.println("ArialBold 14");
  displayStuff();

  tft.setTextColor(RA8875_GREEN);
  tft.setFont(ComicSansMS_14);
  tft.println("ComicSansMS 14");
  displayStuff(); 

  nextPage();
  
  tft.setTextColor(RA8875_WHITE);
  tft.setFont(DroidSans_14);
  tft.println("DroidSans_14");
  displayStuff();

  tft.setTextColor(RA8875_YELLOW);
  tft.setFont(Michroma_14);
  tft.println("Michroma_14");
  displayStuff();

  tft.setTextColor(RA8875_BLACK, RA8875_YELLOW);
  tft.setFont(Crystal_24_Italic);
  tft.println("CRYSTAL_24");
  displayStuff();

  nextPage();

  tft.setTextColor(RA8875_GREEN);
  tft.setFont(Chancery_24_Italic);
  tft.println("Chancery_24_Italic");
  displayStuff();

  //anti-alias font OpenSans
  tft.setTextColor(RA8875_RED, RA8875_YELLOW);
  tft.setFont(OpenSans24);
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

  tft.setTextColor(RA8875_WHITE);
  tft.setFont(Arial_14);
  tft.println("Arial_14");
  displayStuff1();

  tft.setTextColor(RA8875_YELLOW);
  tft.setFont(Arial_14_Bold);
  tft.println("ArialBold 14");
  displayStuff1();

  nextPage();

  tft.setTextColor(RA8875_GREEN);
  tft.setFont(ComicSansMS_14);
  tft.println("ComicSansMS 14");
  displayStuff1(); 

  tft.setTextColor(RA8875_WHITE);
  tft.setFont(DroidSans_14);
  tft.println("DroidSans_14");
  displayStuff1();

  nextPage();

  tft.setTextColor(RA8875_YELLOW);
  tft.setFont(Michroma_14);
  tft.println("Michroma_14");
  displayStuff1();

  nextPage();
  
  tft.setTextColor(RA8875_BLACK, RA8875_YELLOW);
  tft.setFont(Crystal_24_Italic);
  tft.println("CRYSTAL_24");
  displayStuff1();

  tft.setTextColor(RA8875_GREEN);
  tft.setFont(Chancery_24_Italic);
  tft.println("Chancery_24_Italic");
  displayStuff1();
  
  nextPage();

  //anti-alias font OpenSans
  tft.setTextColor(RA8875_RED, RA8875_YELLOW);
  tft.setFont(OpenSans24);
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
    if (i & 1) tft.setTextColor(RA8875_WHITE, RA8875_RED);
    else tft.setTextColor(RA8875_YELLOW, RA8875_BLUE);
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

  tft.fillWindow(RA8875_BLACK);
  tft.setCursor(0, 0);
}
