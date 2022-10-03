/*
  DogLcd Library - PingPong
 
  Demonstrates the basic use two DOGM text LCD displays.
  This sketch prints "Ping" and "Pong" alternately 
  to the displays.
 
  See the hardware documentation for wiring instuctions
  We assume  the following pins are connected:
  * For boths displays SI pin to digital pin 2
  * For boths displays CLK pin to digital pin 3
  * For boths displays RS pin to digital pin 4
  * For the "Ping" display" the CSB pin is connected to pin 5
  * For the "Pong" display" the CSB pin is connected to pin 6
  * For boths displays the RESET pin is not used (connected to +5V)
  * For boths displays the Backlight switching is not used

  Library homepage : http://code.google.com/p/doglcd/

*/

// include the library code:
#include <DogLcd.h>

// Ping : initialize the library with the numbers of the interface pins
DogLcd ping(2, 3, 4, 5);
// Pong : initialize the library with the numbers of the interface pins
DogLcd pong(2, 3, 4, 6);

void setup() {
  // set up the LCD type and the contrast setting for the ping display
  ping.begin(DOG_LCD_M162,0x1F);
  // set up the LCD type and the contrast setting for the pong display 
  pong.begin(DOG_LCD_M162,0x28);
  //clear both displays
  ping.clear();
  pong.clear();
}

void loop() {
  //clear the ping-display
  ping.print("PING");
  delay(1000);
  ping.clear();
  pong.print("PONG");
  delay(1000);
  pong.clear();
}


