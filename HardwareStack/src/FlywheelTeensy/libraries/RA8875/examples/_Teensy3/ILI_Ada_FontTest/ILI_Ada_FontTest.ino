#include <Adafruit_GFX.h>

#include <SPI.h>
#include <RA8875.h>

#include <ili9488_t3_font_ComicSansMS.h>
#include "Fonts/FreeSansOblique12pt7b.h"

#define RA8875_CS 9
#define RA8875_RST 8
RA8875 tft = RA8875(RA8875_CS, RA8875_RST);
uint8_t test_screen_rotation = 0;


void setup() {
  Serial.begin(38400);
  long unsigned debug_start = millis ();
  while (!Serial && ((millis () - debug_start) <= 5000)) ;
  Serial.println("Setup");
  //  begin display: Choose from: RA8875_480x272, RA8875_800x480, RA8875_800x480ALT, Adafruit_480x272, Adafruit_800x480
  tft.begin(RA8875_800x480);

  tft.setRotation(2);
  tft.fillWindow(RA8875_BLACK);

  tft.setTextColor(RA8875_WHITE);
  tft.println("This is a test");
  tft.println(" of the default font");
  tft.println();

  tft.setTextColor(RA8875_WHITE, RA8875_BLUE);
  tft.setFont(ComicSansMS_12);
  tft.println("This is a test!!");
  tft.println(" of ILIxxx font ComicSansMS_12");
  tft.println();

  tft.setTextColor(RA8875_GREEN);
  tft.println("This is Transparent test!!");
  tft.println(" of ILIxxx font ComicSansMS_12");
  tft.println();

  tft.setTextColor(RA8875_WHITE, RA8875_RED);
  tft.setFont(&FreeSansOblique12pt7b);
  tft.println("This is a test of GFX");
  tft.println(" of GFX font FreeSansOblique12pt");
  int cursor_x = tft.getCursorX();
  int cursor_y = tft.getCursorY();
  tft.fillRect(cursor_x, cursor_y, tft.width(), 20, RA8875_YELLOW);
  tft.setTextColor(RA8875_BLUE);
  tft.println("This Transparent GFX");
  tft.println(" of GFX font FreeSansOblique12pt");

  tft.setFontDefault();
  tft.setTextColor(RA8875_GREEN);
  tft.setFontScale(1);
  tft.println("This is default font:");
  tft.setFontSpacing(1);//now give 5 pix extra spacing between chars
  tft.println("ABCDEF 1 2 3 4 5 6 7");
}

void loop()
{
  uint32_t delta_time;
  Serial.println("Press anykey to start speed test");
  while (Serial.read() == -1) ;
  while (Serial.read() != -1) ;

  Serial.printf("\nSpeed test at rotation: %d\n", test_screen_rotation);
  tft.setRotation(test_screen_rotation);
  test_screen_rotation = (test_screen_rotation + 1) & 0x3;

  tft.fillWindow(RA8875_GREEN);
  tft.setCursor(0, 0);
  tft.setTextColor(RA8875_WHITE, RA8875_BLUE);
  tft.setFont(ComicSansMS_12);
  tft.println("Opaque font ComicSansMS_12");
  tft.println();
  delta_time = displayStuff();
  Serial.printf("Opaque font ComicSansMS_12: %u\n", delta_time);
  delay(1000);

  tft.fillWindow(RA8875_YELLOW);
  tft.setCursor(0, 0);
  tft.setTextColor(RA8875_RED);
  tft.println("Transparent ComicSansMS_12");
  tft.println();
  delta_time = displayStuff();
  Serial.printf("Transparent ComicSansMS_12: %u\n", delta_time);
  delay(1000);

  tft.setFont(&FreeSansOblique12pt7b);
  tft.fillWindow(RA8875_BLUE);
  tft.setCursor(0, 0);
  tft.setTextColor(RA8875_WHITE, RA8875_RED);
  tft.println("Opaquefont FreeSansOblique12pt7b");
  tft.println();
  delta_time = displayStuff();
  Serial.printf("Opaquefont FreeSansOblique12pt7b: %u\n",  delta_time);
  delay(1000);

  tft.fillWindow(RA8875_PINK);
  tft.setCursor(0, 0);
  tft.setTextColor(RA8875_BLACK);
  tft.println("Transparent FreeSansOblique12pt7b");
  tft.println();
  delta_time = displayStuff();
  Serial.printf("Transparent FreeSansOblique12pt7b: %u\n",  delta_time);
  delay(1000);
  tft.fillWindow(RA8875_GREEN);
  tft.setCursor(0, 5);
  tft.setTextColor(RA8875_WHITE, RA8875_BLUE);
  tft.setFont(ComicSansMS_20);
  tft.println("Opaque font ComicSansMS_20");
  tft.println();
  delta_time = displayStuff();
  Serial.printf("Opaque font ComicSansMS_20: %u\n", delta_time);
}

uint32_t displayStuff()
{
  elapsedMillis elapsed_time = 0;
  tft.println("ABCDEFGHIJKLMNOPQRSTUVWXYZ");
  tft.println("abcdefghijklmnopqrstuvwxyz");
  tft.println("0123456789");
  tft.println("!@#$%^ &*()-");
  uint32_t return_value = elapsed_time;
  if (Serial.available()) {
    while (Serial.read() != -1) ;
    Serial.println("Test Paused");
    delay(50);
    while (Serial.read() == -1) ;
    while (Serial.read() != -1) ;
  }
  return (uint32_t) return_value;
}
