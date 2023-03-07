/*
  ks0108_Arduino.h - Arduino library support for ks0108 and compatable graphic LCDs
  Copyright (c)2008 Michael Margolis All right reserved

  This is the configuration file for mapping Teensy++ pins to the ks0108 Graphics LCD library
*/

#ifndef	KS0108_CONFIG_H
#define KS0108_CONFIG_H

// default connection - uses most digital pins, leaves most analog pins available
#define CSEL1				7		// CS1 Bit
#define CSEL2				8		// CS2 Bit
#define R_W				6		// R/W Bit
#define D_I				5		// D/I Bit
#define EN				9		// EN Bit
#define LCD_CMD_PORT			PORTD		// pins 5, 6, 7, 8 - all on port D

// alternate connection - uses most analog pins, leaves most digital pins available
//#define CSEL1				17		// CS1 Bit
//#define CSEL2				16		// CS2 Bit
//#define R_W				18		// R/W Bit
//#define D_I				19		// D/I Bit
//#define EN				20		// EN Bit
//#define LCD_CMD_PORT			PORTF		// pins 16-19 - all on port F

// these macros  map pins to ports using the defines above	
// the following should not be changed unless you really know what your doing 
#define LCD_DATA_LOW_NBL   B   // port for low nibble:  pins 0, 1, 2, 3
#define LCD_DATA_HIGH_NBL  B   // port for high nibble: pins 13, 14, 15, 4

// Teensyduino always optimizes digitialWrite when used with const inputs
#define fastWriteHigh(_pin) (digitalWrite((_pin), HIGH))
#define fastWriteLow(_pin)  (digitalWrite((_pin), LOW))

#endif
