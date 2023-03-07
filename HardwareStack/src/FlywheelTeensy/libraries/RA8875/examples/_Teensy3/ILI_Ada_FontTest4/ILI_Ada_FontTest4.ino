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

// maybe a few GFX FOnts?
#include <Fonts/FreeMonoBoldOblique12pt7b.h>
#include <Fonts/FreeSerif12pt7b.h>

typedef struct {
  const ILI9341_t3_font_t *ili_font;
  const GFXfont       *gfx_font;
  const char          *font_name;
  uint16_t            font_fg_color;
  uint16_t            font_bg_color;
} ili_fonts_test_t;


const ili_fonts_test_t font_test_list[] = {
  {nullptr, nullptr,  "Internal Font", RA8875_RED, RA8875_YELLOW},
  {&Arial_14, nullptr,  "Arial_14", RA8875_WHITE, RA8875_WHITE},
  {&Arial_14_Bold, nullptr,  "ArialBold 14", RA8875_YELLOW, RA8875_YELLOW},
  {&ComicSansMS_14, nullptr,  "ComicSansMS 14", RA8875_GREEN, RA8875_GREEN},
  {&DroidSans_14, nullptr,  "DroidSans_14", RA8875_WHITE, RA8875_WHITE},
  {&Michroma_14, nullptr,  "Michroma_14", RA8875_YELLOW, RA8875_YELLOW},
  {&Crystal_24_Italic, nullptr,  "CRYSTAL_24", RA8875_BLACK, RA8875_YELLOW},
  {&Chancery_24_Italic, nullptr,  "Chancery_24_Italic", RA8875_GREEN, RA8875_GREEN},
  {&OpenSans24, nullptr,  "OpenSans 18", RA8875_RED, RA8875_YELLOW},
  {nullptr, &FreeMonoBoldOblique12pt7b,  "GFX FreeMonoBoldOblique12pt7b", RA8875_WHITE, RA8875_WHITE},
  {nullptr, &FreeMonoBoldOblique12pt7b,  "GFX FreeMonoBoldOblique12pt7b", RA8875_RED, RA8875_YELLOW},
  {nullptr, &FreeSerif12pt7b,  "GFX FreeSerif12pt7b", RA8875_WHITE, RA8875_WHITE},
  {nullptr, &FreeSerif12pt7b,  "GFX FreeSerif12pt7b", RA8875_RED, RA8875_YELLOW},

} ;



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
  tft.begin(Adafruit_800x480, 16, 15000000);

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
  tft.fillWindow(RA8875_RED);
  tft.setCursor(CENTER, CENTER);
  tft.printf("Rotation: %d", test_screen_rotation);
  test_screen_rotation = (test_screen_rotation + 1) & 0x3;
  tft.setCursor(200, 300);
  Serial.printf("  Set cursor(200, 300), retrieved(%d %d)",
                tft.getCursorX(), tft.getCursorY());
  tft.setCursor(50, 50);
  tft.write('0');
  tft.setCursor(tft.width() - 50, 50);
  tft.write('1');
  tft.setCursor(50, tft.height() - 50);
  tft.write('2');
  tft.setCursor(tft.width() - 50, tft.height() - 50);
  tft.write('3');

  for (uint8_t font_index = 0; font_index < (sizeof(font_test_list) / sizeof(font_test_list[0])); font_index++) {
    nextPage();
    if (font_test_list[font_index].font_fg_color != font_test_list[font_index].font_bg_color)
      tft.setTextColor(font_test_list[font_index].font_fg_color, font_test_list[font_index].font_bg_color);
    else
      tft.setTextColor(font_test_list[font_index].font_fg_color);
    if (font_test_list[font_index].ili_font) tft.setFont(*font_test_list[font_index].ili_font);
    else if (font_test_list[font_index].gfx_font)  tft.setFont(font_test_list[font_index].gfx_font);
    else tft.setFont(INTFONT);
    tft.println(font_test_list[font_index].font_name);
    Serial.println(font_test_list[font_index].font_name);
    displayStuff1();
  }
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

  int16_t cursorX = tft.getCursorX();
  int16_t cursorY = tft.getCursorY();

  uint16_t width = tft.width();
  uint16_t height = tft.height();
  Serial.printf("DS1 (%d,%d) %d %d\n", cursorX, cursorY, width, height);
  uint16_t rect_x = width / 2 - 50;
  uint16_t rect_y = height - 75;
  tft.drawRect(rect_x, rect_y, 100, 50, RA8875_WHITE);
  for (uint16_t y = rect_y + 5; y < rect_y + 50; y += 5)
    tft.drawFastHLine(rect_x + 1, y, 98, RA8875_PINK);
  for (uint16_t x = rect_x + 5; x < rect_x + 100; x += 5)
    tft.drawFastVLine(x, rect_y+1, 48, RA8875_PINK);
  tft.setCursor(width / 2, height - 50, true);
  tft.print("Center");

  // Lets try again with CENTER X keyword.
  rect_y -= 100;
  tft.drawRect(rect_x, rect_y, 100, 50, RA8875_PINK);
  for (uint16_t y = rect_y + 5; y < rect_y + 50; y += 5)
    tft.drawFastHLine(rect_x + 1, y, 98, RA8875_CYAN);
  for (uint16_t x = rect_x + 5; x < rect_x + 100; x += 5)
    tft.drawFastVLine(x, rect_y+1, 48, RA8875_CYAN);
  tft.setCursor(CENTER, rect_y);
  tft.print("XCENTR");

  // Lets try again with CENTER Y keyword.
  rect_x = 50;
  rect_y = tft.height()/2 -25;
  tft.drawRect(rect_x, rect_y, 100, 50, RA8875_CYAN);
  for (uint16_t y = rect_y + 5; y < rect_y + 50; y += 5)
    tft.drawFastHLine(rect_x + 1, y, 98, RA8875_PINK);
  for (uint16_t x = rect_x + 5; x < rect_x + 100; x += 5)
  tft.setCursor(50, CENTER);
  tft.print("YCENTR");
  
  // Lets see how close the getTextBounds gets the bounds of the text
  rect_x = 200;
  rect_y += 25; //center 
  static const char rectText[] = "RectText";
  int16_t xT, yT; 
  uint16_t wT, hT;
  tft.getTextBounds(rectText, rect_x, rect_y, &xT, &yT, &wT, &hT);
  Serial.printf("getTextBounds: (%d, %d): %d %d %d %d\n", rect_x, rect_y, xT, yT, wT, hT);
  tft.setCursor(rect_x, rect_y);
  tft.print(rectText);
  tft.drawRect(xT, yT, wT, hT, RA8875_CYAN);

  tft.setCursor(cursorX, cursorY);
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