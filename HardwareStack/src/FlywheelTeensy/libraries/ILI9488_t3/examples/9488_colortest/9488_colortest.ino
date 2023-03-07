//=============================================================================
//=============================================================================

#include <ILI9488_t3.h>
#include <ILI9488_t3_font_Arial.h>
#include <ILI9488_t3_font_ArialBold.h>
//#define TEENSY64

#if defined(__MK66FX1M0__) && !defined(TEENSY64)
#define TFT_RST 255
#define TFT_DC 20
#define TFT_CS 21
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
//=============================================================================
// Setup
//=============================================================================
void setup()
{
  tft.begin();
  tft.setRotation(3); // 180
  delay(100);

  tft.fillScreen(ILI9488_BLACK);
  delay(250);
  tft.fillScreen(ILI9488_RED);
  delay(250);
  tft.fillScreen(ILI9488_BLUE);
  delay(250);
  tft.fillScreen(ILI9488_GREEN);
  delay(250);
  tft.fillScreen(ILI9488_BLACK);


}

void drawColor(uint16_t x, uint16_t y, const char *psz, uint16_t color)
{
  tft.setFontAdafruit();
  tft.setTextColor(color);
  tft.setTextSize(2);
  tft.setCursor(x, y);
  tft.print(psz);
  tft.drawRect(x + 100, y, 50, 50, color);
  tft.fillRect(x + 110, y + 10, 30, 30, color);
  tft.drawLine(x + 100, y + 70, x + 200, y + 70, color);
  tft.drawLine(x + 220, y, x + 220, y + 70, color);
  tft.drawLine(x + 100, y + 70, x + 220, y, color);
  tft.drawCircle(x + 50, y + 50, 28, color);
  tft.fillCircle(x + 50, y + 50, 20, color);
  tft.setFont(Arial_12_Bold);
  tft.setCursor(x + 160, y + 50);
  tft.print(psz);
}


//=============================================================================
// Loop
//=============================================================================
bool use_frame_buffer = false;
void loop()
{
  tft.useFrameBuffer(use_frame_buffer);
  tft.setFont(Arial_18_Bold);
  tft.setCursor(0, 150);
  if (use_frame_buffer) {
    tft.fillScreen(ILI9488_GREENYELLOW);
  } else {
    tft.fillScreen(ILI9488_BLACK);
  }
  drawColor(0, 0, "Red", ILI9488_RED);
  drawColor(0, 80, "Green", ILI9488_GREEN);
  drawColor(0, 160, "Blue", ILI9488_BLUE);
  drawColor(0, 240, "White", ILI9488_WHITE);

  drawColor(240, 0, "Yellow", ILI9488_YELLOW);
  drawColor(240, 80, "Orange", ILI9488_ORANGE);
  drawColor(240, 160, "Cyan", ILI9488_CYAN);
  drawColor(240, 240, "Pink", ILI9488_PINK);
  if (use_frame_buffer) {
    tft.updateScreen();
    use_frame_buffer = false;
  } else {
    use_frame_buffer = true;
  }
  delay(2500);
}
