//-------------------------------------------------------------------
// Kurt's Frame buffer and clip tests
//
// This test program is a set of random tests that have been done
// over time to test out the different functions to make sure they
// are working with the clipping functions as well as the frame
// buffer.  So you can for example test to see if you get the 
// same results with the frame buffer turned on or off
// 
// this sketch is in the public domain. 
//
// This sketch depends on the fonts that are contained in the library
//     https://github.com/mjs513/ILI9341_fonts
//-------------------------------------------------------------------

//#define TRY_EXTMEM
#ifdef TRY_EXTMEM
#if !defined(ARDUINO_TEENSY41)
#undef TRY_EXTMEM
#if defined(ENABLE_EXT_DMA_UPDATES)
#error "This Version only works with Teensy 4.1 with External memory"
#endif
#endif
#endif

#include <Adafruit_GFX.h>    // Core graphics library
#include <ili9488_t3_font_Arial.h>
#include <ili9488_t3_font_ArialBold.h>
#include <ILI9488_t3.h>
#include <Fonts/FreeMonoBoldOblique12pt7b.h>
#include <Fonts/FreeSerif12pt7b.h>

#define ROTATION 3

#include "SPI.h"

//#define USE_SPI1
#if defined(USE_SPI1)
#if defined(__IMXRT1062__)  // Teensy 4.x 
#define TFT_DC 2
#define TFT_CS  0
#define TFT_RST 3

#define TFT_SCK 27
#define TFT_MISO 1
#define TFT_MOSI 26

#else
#define TFT_DC 31
#define TFT_CS 10 // any pin will work not hardware
#define TFT_RST 8
#define TFT_SCK 32
#define TFT_MISO 5
#define TFT_MOSI 21
//#define DEBUG_PIN 13
#endif
ILI9488_t3 tft = ILI9488_t3(&SPI1, TFT_CS, TFT_DC, TFT_RST, TFT_MOSI, TFT_SCK, TFT_MISO);

//------------------------------------
#else // default pins
#define TFT_DC  9  // only CS pin 
#define TFT_CS 10   // using standard pin
#define TFT_RST 8
ILI9488_t3 tft = ILI9488_t3(&SPI, TFT_CS, TFT_DC, TFT_RST);
//--------------------------------------
#endif

#define DBGSerial Serial

uint16_t our_pallet[] = {
  ILI9488_BLACK,  ILI9488_RED, ILI9488_GREEN,  ILI9488_BLUE,   ILI9488_WHITE,
  ILI9488_YELLOW, ILI9488_ORANGE, ILI9488_CYAN, ILI9488_PINK
};

Adafruit_GFX_Button button;

// Let's allocate the frame buffer ourself.
// BUGBUG: IF RAFB is 4 this won't fit
#if ! defined(ENABLE_EXT_DMA_UPDATES)
DMAMEM RAFB tft_frame_buffer[ILI9488_TFTWIDTH * ILI9488_TFTHEIGHT];
#endif


#ifdef TRY_EXTMEM
EXTMEM RAFB extmem_frame_buffer[ILI9488_TFTWIDTH * ILI9488_TFTHEIGHT];
#endif

uint8_t use_extmem = 0;
uint8_t use_dma = 0;
uint8_t use_clip_rect = 0;
uint8_t use_set_origin = 0;
uint8_t use_fb = 0;

#define ORIGIN_TEST_X 50
#define ORIGIN_TEST_Y 50


void setup() {
  while (!Serial && (millis() < 4000)) ;
  DBGSerial.begin(115200);
  DBGSerial.printf("Begin: CS:%d, DC:%dRST: %d\n", TFT_CS, TFT_DC, TFT_RST);
  DBGSerial.printf("  Size of RAFB: %d\n", sizeof(RAFB));
  tft.begin(26000000);

#if defined(ENABLE_EXT_DMA_UPDATES)
  tft.setFrameBuffer(extmem_frame_buffer);
#else
  tft.setFrameBuffer(tft_frame_buffer);
#endif
  tft.setRotation(ROTATION);
  tft.fillScreen(ILI9488_BLACK);

  // read diagnostics (optional but can help debug problems)
  uint8_t x = tft.readcommand8(ILI9488_RDMODE);
  DBGSerial.print("Display Power Mode: 0x"); DBGSerial.println(x, HEX);
  x = tft.readcommand8(ILI9488_RDMADCTL);
  DBGSerial.print("MADCTL Mode: 0x"); DBGSerial.println(x, HEX);
  x = tft.readcommand8(ILI9488_RDPIXFMT);
  DBGSerial.print("Pixel Format: 0x"); DBGSerial.println(x, HEX);
  x = tft.readcommand8(ILI9488_RDIMGFMT);
  DBGSerial.print("Image Format: 0x"); DBGSerial.println(x, HEX);
  x = tft.readcommand8(ILI9488_RDSELFDIAG);
  DBGSerial.print("Self Diagnostic: 0x"); DBGSerial.println(x, HEX);
#ifdef DEBUG_PIN
  pinMode(DEBUG_PIN, OUTPUT);
#endif

  button.initButton(&tft, 200, 125, 100, 40, ILI9488_GREEN, ILI9488_YELLOW, ILI9488_RED, "UP", 1, 1);
  DBGSerial.println("Just before frist draw Test Screen");
#ifdef TRY_EXTMEM
  testEXTMem();
#endif

  drawTestScreen();
}

void SetupOrClearClipRectAndOffsets() {
  if (use_clip_rect) {
    tft.setClipRect();  // make sure we clear the whole screen
    tft.setOrigin();    // make sure none are set yet

    tft.fillScreen(ILI9488_LIGHTGREY);

    // Now lets set origin.
    if (use_set_origin)
      tft.setOrigin(ORIGIN_TEST_X, ORIGIN_TEST_Y);
    int x = tft.width() / 4;
    int y = tft.height() / 4;
    int w = tft.width() / 2;
    int h = tft.height() / 2;
    tft.drawRect(x, y, w, h, ILI9488_ORANGE);
    tft.updateScreen();
    tft.setClipRect(x + 1, y + 1, w - 2, h - 2);
    delay(250);

  } else {
    tft.setClipRect();
    if (use_set_origin)
      tft.setOrigin(ORIGIN_TEST_X, ORIGIN_TEST_Y);
    else
      tft.setOrigin();
  }
}


uint16_t palette[256];  // Should probably be 256, but I don't use many colors...
uint16_t pixel_data[2500];
const uint8_t pict1bpp[] = {0xff, 0xff, 0xc0, 0x03, 0xa0, 0x05, 0x90, 0x9, 0x88, 0x11, 0x84, 0x21, 0x82, 0x41, 0x81, 0x81,
                            0x81, 0x81, 0x82, 0x41, 0x84, 0x21, 0x88, 0x11, 0x90, 0x09, 0xa0, 0x05, 0xc0, 0x03, 0xff, 0xff
                           };
const uint8_t pict2bpp[] = {
  0x00, 0x00, 0x55, 0x55, 0xaa, 0xaa, 0xff, 0xff,
  0x00, 0x00, 0x55, 0x55, 0xaa, 0xaa, 0xff, 0xff,
  0x55, 0x55, 0xaa, 0xaa, 0xff, 0xff, 0x00, 0x00,
  0x55, 0x55, 0xaa, 0xaa, 0xff, 0xff, 0x00, 0x00,
  0xaa, 0xaa, 0xff, 0xff, 0x00, 0x00, 0x55, 0x55,
  0xaa, 0xaa, 0xff, 0xff, 0x00, 0x00, 0x55, 0x55,
  0xff, 0xff, 0x00, 0x00, 0x55, 0x55, 0xaa, 0xaa,
  0xff, 0xff, 0x00, 0x00, 0x55, 0x55, 0xaa, 0xaa,
  0x00, 0x00, 0x55, 0x55, 0xaa, 0xaa, 0xff, 0xff,
  0x00, 0x00, 0x55, 0x55, 0xaa, 0xaa, 0xff, 0xff,
  0x55, 0x55, 0xaa, 0xaa, 0xff, 0xff, 0x00, 0x00,
  0x55, 0x55, 0xaa, 0xaa, 0xff, 0xff, 0x00, 0x00,
  0xaa, 0xaa, 0xff, 0xff, 0x00, 0x00, 0x55, 0x55,
  0xaa, 0xaa, 0xff, 0xff, 0x00, 0x00, 0x55, 0x55,
  0xff, 0xff, 0x00, 0x00, 0x55, 0x55, 0xaa, 0xaa,
  0xff, 0xff, 0x00, 0x00, 0x55, 0x55, 0xaa, 0xaa,
};
const uint8_t pict4bpp[] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x00,
  0x00, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x00,
  0x00, 0x11, 0x22, 0x22, 0x22, 0x22, 0x11, 0x00,
  0x00, 0x11, 0x22, 0x22, 0x22, 0x22, 0x11, 0x00,
  0x00, 0x11, 0x22, 0x33, 0x33, 0x22, 0x11, 0x00,
  0x00, 0x11, 0x22, 0x33, 0x33, 0x22, 0x11, 0x00,
  0x00, 0x11, 0x22, 0x33, 0x33, 0x22, 0x11, 0x00,
  0x00, 0x11, 0x22, 0x33, 0x33, 0x22, 0x11, 0x00,
  0x00, 0x11, 0x22, 0x22, 0x22, 0x22, 0x11, 0x00,
  0x00, 0x11, 0x22, 0x22, 0x22, 0x22, 0x11, 0x00,
  0x00, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x00,
  0x00, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};



void drawTestScreen() {
  DBGSerial.printf("Use FB: %d ", use_fb); DBGSerial.flush();
  tft.useFrameBuffer(use_fb);
  SetupOrClearClipRectAndOffsets();
  uint32_t start_time = millis();
  tft.fillScreen(use_fb ? ILI9488_RED : ILI9488_BLACK);
  //tft.setFont(Inconsolata_60);
  tft.setFont(Arial_24_Bold);
  tft.setTextColor(ILI9488_WHITE);
  tft.setCursor(0, 0);
  tft.println("Test");
  tft.setTextColor(ILI9488_WHITE, ILI9488_RED);
  tft.println("text");
  tft.setCursor(85, 65);
  tft.print("XYZ");
  tft.setFontAdafruit();
  tft.setTextSize(2);
  tft.setTextColor(ILI9488_WHITE);
  tft.println("01234");
  tft.setTextColor(ILI9488_WHITE, ILI9488_GREEN);
  tft.println("56789!@#$%");

  tft.drawRect(0, 150, 100, 50, ILI9488_WHITE);
  tft.drawLine(0, 150, 100, 50, ILI9488_GREEN);
  tft.fillRectVGradient(125, 150, 50, 50, ILI9488_GREEN, ILI9488_YELLOW);
  tft.fillRectHGradient(200, 150, 50, 50, ILI9488_YELLOW, ILI9488_GREEN);
  // Try a read rect and write rect
#ifdef DEBUG_PIN
  digitalWrite(DEBUG_PIN, HIGH);
#endif

  tft.readRect(0, 0, 50, 50, pixel_data);
  // For heck of it lets make sure readPixel and ReadRect
  // give us same data, maybe check along diagnal?
  for (uint i = 0; i < 50; i++) {
    uint16_t pixel_color = tft.readPixel(i, i);
    if (pixel_color != pixel_data[i * 50 + i]) {
      DBGSerial.printf("Read rect/pixel mismatch: %d %x %x\n", i, pixel_color, pixel_data[i * 50 + i]);
    }
  }

#ifdef DEBUG_PIN
  digitalWrite(DEBUG_PIN, LOW);
#endif
  tft.writeRect(250, 0, 50, 50, pixel_data);

  // Lets try to pack this rectangle of data into 8 byte
  tft.readRect(85, 65, 50, 50, pixel_data);
  uint16_t *ppd16 = pixel_data;
  uint8_t *ppd8 = (uint8_t*)pixel_data;
  uint8_t palette_cnt = 0;
  int palette_index;
  for (int i = 0; i < 2500; i++) {
    for (palette_index = 0; palette_index < palette_cnt; palette_index++) {
      if (*ppd16 == palette[palette_index])
        break;
    }
    if (palette_index >= palette_cnt) {
      palette[palette_cnt++] = *ppd16;  // save away the color
    }
    *ppd8++ = palette_index;
    ppd16++;
  }
  tft.writeRect8BPP(200, 50, 50, 50, (uint8_t*)pixel_data, palette);
  palette[0] = ILI9488_CYAN;
  palette[1] = ILI9488_OLIVE;
  tft.writeRect1BPP(75, 100, 16, 16, pict1bpp, palette);
  tft.writeRect1BPP(320 - 90, 75, 16, 16, pict1bpp, palette);

  palette[2] = ILI9488_MAROON;
  palette[3] = ILI9488_PINK;
  tft.writeRect2BPP(75, 125, 32, 16, pict2bpp, palette);

  tft.writeRectNBPP(15, 125, 32, 16, 2, pict2bpp, palette);
  tft.writeRectNBPP(75, 150, 16, 16, 4, pict4bpp, palette);

  // Try drawing button
  tft.setFontAdafruit();
  button.drawButton();

  // Lets fill up some more of the larger screen.
  tft.fillCircle(380, 220, 80, ILI9488_GREEN);
  tft.fillCircle(380, 220, 60, ILI9488_BLUE);
  tft.drawCircle(380, 220, 40, ILI9488_RED);
  tft.drawCircle(380, 220, 20, ILI9488_YELLOW);

  tft.fillTriangle(20, 300, 170, 300, 95, 240, ILI9488_GREEN);
  tft.fillTriangle(40, 280, 150, 280, 95, 220, ILI9488_RED);
  tft.drawTriangle(60, 260, 130, 260, 95, 200, ILI9488_YELLOW);
  tft.drawTriangle(80, 240, 110, 240, 95, 180, ILI9488_BLUE);

  tft.setFont(&FreeMonoBoldOblique12pt7b);
  tft.setCursor(250, 50);
  tft.setTextColor(ILI9488_WHITE);
  tft.println("Adafruit");
  tft.setCursor(250, tft.getCursorY());
  tft.setTextColor(ILI9488_WHITE, ILI9488_GREEN);
  tft.println("MonoBold");


  if (use_dma) {
    tft.updateScreenAsync();
  } else {
    tft.updateScreen();
  }
  DBGSerial.println(millis() - start_time, DEC);

  if (use_dma && use_fb) {
    delay(500);
    DBGSerial.printf("DMA error status: %x\n", DMA_ES);
  }

  use_fb = use_fb ? 0 : 1 ;
  DBGSerial.println(use_fb, DEC);


}

void fillScreenTest() {
  tft.useFrameBuffer(0);
  SetupOrClearClipRectAndOffsets();

  tft.fillScreen(ILI9488_RED);
  WaitForUserInput();
  tft.fillScreen(ILI9488_GREEN);
  WaitForUserInput();
  tft.fillScreen(ILI9488_WHITE);
  WaitForUserInput();
  tft.fillScreen(ILI9488_BLACK);

}
void printTextSizes(const char *sz) {
  Serial.printf("%s(%d,%d): SPL:%u ", sz, tft.getCursorX(), tft.getCursorY(), 
      tft.strPixelLen(sz, strlen(sz)));
  int16_t x, y;
  uint16_t w, h;
  tft.getTextBounds(sz, tft.getCursorX(), tft.getCursorY(), &x, &y, &w, &h);
  Serial.printf(" Rect(%d, %d, %u %u)\n", x, y, w, h);
  tft.drawRect(x, y, w, h, ILI9488_GREEN);
}


void drawTextScreen(bool fOpaque) {
  SetupOrClearClipRectAndOffsets();
  tft.setTextSize(1);
  uint32_t start_time = millis();
  tft.useFrameBuffer(use_fb);
  tft.fillScreen(use_fb ? ILI9488_RED : ILI9488_BLACK);
  tft.setFont(Arial_40_Bold);
  if (fOpaque)
    tft.setTextColor(ILI9488_WHITE, use_fb ? ILI9488_BLACK : ILI9488_RED);
  else
    tft.setTextColor(ILI9488_WHITE);
  tft.setCursor(0, 5);
  tft.println("AbCdEfGhIj");
  tft.setFont(Arial_28_Bold);
  tft.println("0123456789!@#$");
#if 0
  tft.setFont(Arial_20_Bold);
  tft.println("abcdefghijklmnopq");
  tft.setFont(Arial_14_Bold);
  tft.println("ABCDEFGHIJKLMNOPQRST");
  tft.setFont(Arial_10_Bold);
  tft.println("0123456789zyxwvutu");
#endif
  tft.setFont(&FreeMonoBoldOblique12pt7b);
  printTextSizes("AdaFruit");
  tft.println("yAdaFruitq");
  tft.setFont(&FreeSerif12pt7b);
  tft.setTextSize(2);
  printTextSizes("FreeSan12");
  tft.println("FreeSan12q");
  tft.setFont();
  tft.setTextSize(1, 2);
  printTextSizes("Sys(1,2)");
  tft.println("Sys(1,2)");
  tft.setTextSize(1);
  printTextSizes("System");
  tft.println("System");
  tft.setTextSize(1);


  tft.updateScreen();
  DBGSerial.printf("Use FB: %d OP: %d, DT: %d OR: %d\n", use_fb, fOpaque, use_set_origin, millis() - start_time);
}
//=============================================================================
// Wait for user input
//=============================================================================
void WaitForUserInput() {
  DBGSerial.println("Hit Enter to continue");
  DBGSerial.flush();
  while (DBGSerial.read() == -1) ;
  while (DBGSerial.read() != -1) ;

}

//=============================================================================
// Try continuous update
//=============================================================================
void WaitForFrame(bool fCont, uint32_t wait_frame_count) {
  if (fCont) {
    while (tft.frameCount() < wait_frame_count) yield();
  } else {
    tft.updateScreenAsync();
    WaitForUserInput();
  }
}

void testDMAContUpdate(bool fCont) {
  // Force frame buffer on

  DBGSerial.printf("continuous DMA udpate test - Frame mode on\n"); DBGSerial.flush();
  if (!fCont) {
    DBGSerial.println("Step Mode");
    DBGSerial.flush();
  }
  use_fb = 1; //

  tft.useFrameBuffer(use_fb);
  tft.fillScreen(ILI9488_GREEN);

  // check to see if screen memory actually turned green.
  if (use_fb) {
    RAFB *pw = tft.getFrameBuffer();
    RAFB green_pallet_index = tft.mapColorToPalletIndex(ILI9488_GREEN);
    int error_count = 0;
    for (int i = 0; i < (ILI9488_TFTWIDTH * ILI9488_TFTHEIGHT); i++)
    {
      if (*pw != green_pallet_index) {
        if (error_count < 10) DBGSerial.printf("tft.fillScreen(ILI9488_GREEN) not green? %d %x != %x\n", i, *pw, green_pallet_index);
        error_count++;
      }
      pw++;
    }
    DBGSerial.printf("tft.fillScreen(ILI9488_GREEN(%x)) error count = %d\n", ILI9488_GREEN, error_count);
  }

  if (fCont)
    tft.updateScreenAsync(fCont);

  // Start the update
  WaitForFrame(fCont, 10);

  tft.fillScreen(ILI9488_YELLOW);
  tft.drawRect(5, 5, 310, 230, ILI9488_GREEN);
  tft.fillRect(140, 100, 40, 40, ILI9488_BLUE);
  WaitForFrame(fCont, 20);

  tft.fillScreen(ILI9488_RED);
  tft.drawRect(5, 5, 310, 230, ILI9488_WHITE);

  WaitForFrame(fCont, 30);

  tft.fillScreen(ILI9488_BLACK);

  tft.drawRect(5, 5, 310, 230, ILI9488_GREEN);
  tft.drawRect(25, 25, 270, 190, ILI9488_RED);
  WaitForFrame(fCont, 40);

  digitalWrite(0, HIGH);
  tft.drawRect(5, 5, 310, 230, ILI9488_GREEN);
  tft.setCursor(10, 100);
  tft.setTextColor(ILI9488_RED, ILI9488_BLACK);
  tft.setFont(Arial_20_Bold);
  tft.println("DONE");
  tft.setFont();
  tft.setCursor(10, 200);
  tft.setTextColor(ILI9488_GREEN);
  tft.print("Done");
  tft.setTextSize(2);
  tft.setCursor(10, 50);
  tft.setTextColor(ILI9488_WHITE, ILI9488_RED);
  tft.print("Done");
  digitalWrite(0, LOW);
  WaitForFrame(fCont, 45);
  tft.fillRect(0, 0, 2, 2, ILI9488_PURPLE);

  if (!fCont) {
    DBGSerial.println("Lets now try doing Continue for a few iterations to see if it changes");
    tft.updateScreenAsync(true);
    while (tft.frameCount() < 10) yield();
  }
  tft.endUpdateAsync();
  DBGSerial.println("after endUpdateAsync");
  tft.waitUpdateAsyncComplete();
  DBGSerial.println("after waitUpdateAsyncComplete");
  DBGSerial.println("Finished test");

  delay(2000);
  DBGSerial.println("Do normal update to see if data is there");
  tft.updateScreen();

}

void loop(void) {
  // See if any text entered
  int ich;
  if ((ich = DBGSerial.read()) != -1) {
    while (DBGSerial.read() != -1) delay(1);

    // See if We have a dma operation in progress?
    if (tft.asyncUpdateActive()) {
      DBGSerial.printf("Async Update active DMA error status: %x\n", DMA_ES);
      //tft.dumpDMASettings();
    }

    if (ich == 'c') {
      use_clip_rect = !use_clip_rect;
      if (use_clip_rect) DBGSerial.println("Clip Rectangle Turned on");
      else DBGSerial.println("Clip Rectangle turned off");
      return;
    }
    if (ich == 'd') {
      use_dma = !use_dma;
      if (use_dma) DBGSerial.println("DMA Turned on");
      else DBGSerial.println("DMA turned off");
      return;
    }
#ifdef TRY_EXTMEM
#if ! defined(ENABLE_EXT_DMA_UPDATES)
    if (ich == 'e') {
      use_extmem = !use_extmem;
      if (use_extmem) {
        tft.setFrameBuffer(extmem_frame_buffer);
        DBGSerial.println("Using External memory");
      } else  {
        tft.setFrameBuffer(tft_frame_buffer);
        DBGSerial.println("Using DMAMEM");
      }
      return;
    }
#endif
#endif
    if (ich == 's') {
      use_set_origin = !use_set_origin;
      if (use_set_origin) DBGSerial.printf("Set origin to %d, %d\n", ORIGIN_TEST_X, ORIGIN_TEST_Y);
      else DBGSerial.println("Clear origin");
      return;
    }

    if (ich == 'o')
      drawTextScreen1(1);
    else if (ich == 'f')
      fillScreenTest();
    else if (ich == 't')
      drawTextScreen1(0);
    else if (ich == 'r') {
      testDMAContUpdate(true);
      DBGSerial.println("Returned from testDMAContUpdate");
    }
    else if (ich == 'a') {
      testDMAContUpdate(false);
      DBGSerial.println("Returned from testDMAContUpdate");
    }
    else
      drawTestScreen();
  }
}

void drawTextScreen1(bool fOpaque) {
  SetupOrClearClipRectAndOffsets();
  tft.setTextSize(1);
  uint32_t start_time = millis();
  tft.useFrameBuffer(use_fb);
  tft.fillScreen(use_fb ? ILI9488_RED : ILI9488_BLACK);
  tft.setFont(Arial_28_Bold);
  //t  tft.setFont(Arial_40_Bold);
  if (fOpaque)
    tft.setTextColor(ILI9488_WHITE, use_fb ? ILI9488_BLACK : ILI9488_RED);
  else
    tft.setTextColor(ILI9488_WHITE);
  tft.setCursor(0, 5);
  tft.println("AbCdEfGhIj");
#if 0
  tft.setFont(Arial_28_Bold);
  tft.println("0123456789!@#$");
  tft.setFont(Arial_20_Bold);
  tft.println("abcdefghijklmnopq");
  tft.setFont(Arial_14_Bold);
  tft.println("ABCDEFGHIJKLMNOPQRST");
  tft.setFont(Arial_10_Bold);
  tft.println("0123456789zyxwvutu");
#endif
  tft.setFont(&FreeMonoBoldOblique12pt7b);
  printTextSizes("AdaFruit_MB_12");
  if (fOpaque) {
    tft.setTextColor(ILI9488_RED, ILI9488_BLUE);
    tft.print("A");
    tft.setTextColor(ILI9488_WHITE, ILI9488_GREEN);
    tft.print("d");
    tft.setTextColor(ILI9488_RED, ILI9488_BLUE);
    tft.print("a");
    tft.setTextColor(ILI9488_WHITE, ILI9488_GREEN);
    tft.print("F");
    tft.setTextColor(ILI9488_RED, ILI9488_BLUE);
    tft.print("R");
    tft.setTextColor(ILI9488_WHITE, ILI9488_GREEN);
    tft.print("u");
    tft.setTextColor(ILI9488_RED, ILI9488_BLUE);
    tft.print("i");
    tft.setTextColor(ILI9488_WHITE, ILI9488_GREEN);
    tft.print("t");
    tft.setTextColor(ILI9488_RED, ILI9488_BLUE);
    tft.print("_");
    tft.setTextColor(ILI9488_WHITE, ILI9488_GREEN);
    tft.print("M");
    tft.setTextColor(ILI9488_RED, ILI9488_BLUE);
    tft.print("B");
    tft.setTextColor(ILI9488_WHITE, ILI9488_GREEN);
    tft.print("_");
    tft.setTextColor(ILI9488_RED, ILI9488_BLUE);
    tft.print("1");
    tft.setTextColor(ILI9488_WHITE, ILI9488_GREEN);
    tft.println("2");
    tft.setTextColor(ILI9488_WHITE, use_fb ? ILI9488_BLACK : ILI9488_RED);
  }
  else tft.println("AdaFruit_MB_12");
  tft.setFont(&FreeSerif12pt7b);
  printTextSizes("FreeSan12");
  tft.println("FreeSan12");
  tft.println();
  tft.setTextSize(1, 3);
  printTextSizes("Size 1,3");
  tft.println("Size 1,3");
  tft.setFont();
  tft.setCursor(0, 190);
  tft.setTextSize(1, 2);
  printTextSizes("Sys(1,2)");
  tft.println("Sys(1,2)");
  tft.setTextSize(1);
  printTextSizes("System");
  tft.println("System");
  tft.setTextSize(1);


  tft.updateScreen();
  DBGSerial.printf("Use FB: %d OP: %d, DT: %d OR: %d\n", use_fb, fOpaque, use_set_origin, millis() - start_time);
}

#ifdef TRY_EXTMEM
void  testEXTMem() {
  elapsedMicros em;
#if defined(ENABLE_EXT_DMA_UPDATES)
  em = 0;
  tft.setFrameBuffer(extmem_frame_buffer);
  tft.fillScreen(ILI9488_RED);
  tft.updateScreen();
  DBGSerial.printf("EXT(%x) %d\n",  (uint32_t)extmem_frame_buffer, (uint32_t)em);
#else
  tft.useFrameBuffer(true);
  // Lets see DMAMEM versus EXTMEM
  for (uint8_t i = 0; i < 5; i++) {
    em = 0;
    tft.setFrameBuffer(tft_frame_buffer);
    tft.fillScreen(ILI9488_GREEN);
    tft.updateScreen();
    DBGSerial.printf("DMA(%x) %d\n", (uint32_t)tft_frame_buffer, (uint32_t)em);
    delay(250);
    em = 0;
    tft.setFrameBuffer(extmem_frame_buffer);
    tft.fillScreen(ILI9488_RED);
    tft.updateScreen();
    DBGSerial.printf("EXT(%x) %d\n",  (uint32_t)extmem_frame_buffer, (uint32_t)em);
    delay(250);
  }
  WaitForUserInput();
  tft.useFrameBuffer(false);
  tft.setFrameBuffer(tft_frame_buffer);
#endif
}
#endif