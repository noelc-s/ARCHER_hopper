/*
  ks0108_Arduino.h - Arduino library support for ks0108 and compatable graphic LCDs
  Copyright (c)2008 Michael Margolis All right reserved

  This is the configuration file for mapping Teensy++ pins to the ks0108 Graphics LCD library
*/

#ifndef	KS0108_CONFIG_H
#define KS0108_CONFIG_H

#define CSEL1				18		// CS1 Bit   
#define CSEL2				19		// CS2 Bit
#define R_W				8		// R/W Bit
#define D_I				9		// D/I Bit 
#define EN				7		// EN Bit
#define LCD_CMD_PORT			PORTE		// pins 8, 9, 18, 19

// these macros  map pins to ports using the defines above	
// the following should not be changed unless you really know what your doing 
#define LCD_DATA_LOW_NBL   C   // port for low nibble:  pins 10-13
#define LCD_DATA_HIGH_NBL  C   // port for high nibble: pins 14-17

// Teensyduino always optimizes digitialWrite when used with const inputs
#define fastWriteHigh(_pin) (digitalWrite((_pin), HIGH))
#define fastWriteLow(_pin)  (digitalWrite((_pin), LOW))

#endif
