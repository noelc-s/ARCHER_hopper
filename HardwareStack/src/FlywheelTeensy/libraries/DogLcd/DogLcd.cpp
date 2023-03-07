/*
 * This is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * DogLcd is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with DogLcd.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2010 Eberhard Fahle <e.fahle@wayoda.org>
 */
#include "DogLcd.h"

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

DogLcd::DogLcd(int lcdSI, int lcdCLK, int lcdRS, int lcdCSB, int lcdRESET, int backLight) {
    this->lcdSI=lcdSI;
    this->lcdCLK=lcdCLK;
    this->lcdRS=lcdRS;
    this->lcdCSB=lcdCSB;
    this->lcdRESET=lcdRESET;
    this->backLight=backLight;
}

int DogLcd::begin(int model, int contrast, int vcc) {
    //init all pins to go HIGH, we dont want to send any commands by accident
    pinMode(this->lcdCSB,OUTPUT);
    digitalWrite(this->lcdCSB,HIGH);
    pinMode(this->lcdSI,OUTPUT);
    digitalWrite(this->lcdSI,HIGH);
    pinMode(this->lcdCLK,OUTPUT);
    digitalWrite(this->lcdCLK,HIGH);
    pinMode(this->lcdRS,OUTPUT);
    digitalWrite(this->lcdRS,HIGH);
    if(this->lcdRESET!=-1) {
	pinMode(this->lcdRESET,OUTPUT);
	digitalWrite(this->lcdRESET,HIGH);
    }
    if(this->backLight!=-1) {
	pinMode(this->backLight,OUTPUT);
	digitalWrite(this->backLight,LOW);
    }
    if(model==DOG_LCD_M081) {
	this->model=model;
	rows=1;
	cols=8;
	memSize=80;
	startAddress[0]=0;
	startAddress[1]=-1;
	startAddress[2]=-1;
    }
    else if(model==DOG_LCD_M162) {
	this->model=model;
	rows=2;
	cols=16;
	memSize=40;
	startAddress[0]=0;
	startAddress[1]=0x40;
	startAddress[2]=-1;
    }
    else if(model==DOG_LCD_M163) {
	this->model=model;
	rows=3;
	cols=16;
	memSize=16;
	startAddress[0]=0;
	startAddress[1]=0x10;
	startAddress[2]=0x20;
    }
    else {
	//unknown or unsupported model
	return -1;
    }
    if(contrast < 0 || contrast> 0x3F) {
	//contrast is outside the valid range
	return -1;
    }
    this->contrast=contrast;
    if(vcc==DOG_LCD_VCC_5V || vcc==DOG_LCD_VCC_3V3) {
	this->vcc=vcc;
    }
    else {
	//unknown or unsupported supply voltage
	return -1;
    }
    reset();
    return 0;
}

void DogLcd::reset() {
    if(lcdRESET!=-1) {
	//If user wired the reset line, pull it low and wait for 40 millis
	digitalWrite(lcdRESET,LOW);
	delay(40);
	digitalWrite(lcdRESET,HIGH);
	delay(40);
    }
    else {
	//User wants software reset, we simply wait a bit for stable power
	delay(50);
    }
    if(model==DOG_LCD_M081) {
	//8-bit,1-line
	instructionSetTemplate=(uint8_t)0x30;
    }
    else if(model==DOG_LCD_M162) {
	//8-bit,2-line
	instructionSetTemplate=(uint8_t)0x38;
    }
    else if(model==DOG_LCD_M163) {
	//8-bit,2-line
	instructionSetTemplate=(uint8_t)0x38;
    }
    //init data-size and lines, then change to instructionset 1
    setInstructionSet(1);
    //bias 1/4
    writeCommand(0x1D,30);
    setContrast(this->contrast);
    //Standard setting is : display on, cursor on, no blink
    displayMode=0x04;
    cursorMode=0x02;
    blinkMode=0x00;
    writeDisplayMode();
    entryMode=0x04;
    clear();
    leftToRight();
}

void DogLcd::setContrast(int contrast) {
    if(contrast<0 || contrast>0x3F)
	return;
    if(this->vcc==DOG_LCD_VCC_5V) { 
	/*
	  For 5v operation the booster must be off, which is on the 
	  same command as the (2-bit) high-nibble of contrast
	*/
	writeCommand((0x50 | ((contrast>>4)&0x03)),30);
	/* Set amplification ratio for the follower control */
	writeCommand(0x69,30);
    }
    else {
	/*
	  For 3.3v operation the booster must be on, which is on the 
	  same command as the (2-bit) high-nibble of contrast
	*/
	writeCommand((0x54 | ((contrast>>4)&0x03)),30);
	/* 
	   Set amplification ratio for the follower control
	   this has to be higher it seems at 3.3V operation
	 */
	writeCommand(0x6B,30);
    }	
    //set low-nibble of the contrast
    writeCommand((0x70 | (contrast & 0x0F)),30);
}    
    
void DogLcd::clear() {
    writeCommand(0x01,1080);
}

void DogLcd::home() {
    writeCommand(0x02,1080);
}

void DogLcd::setCursor(int col, int row) {
    if(col>=memSize || row>=rows) {
	//not a valid cursor position
	return;
    }
    int address=(startAddress[row]+col) & 0x7F;
    writeCommand(0x80|address,30);
}

void DogLcd::noDisplay() {
    displayMode=0x00;
    writeDisplayMode();
}

void DogLcd::display() {
    displayMode=0x04;
    writeDisplayMode();
}

void DogLcd::noCursor() {
    cursorMode=0x00;
    writeDisplayMode();
}

void DogLcd::cursor() {
    cursorMode=0x02;
    writeDisplayMode();
}

void DogLcd::noBlink() {
    blinkMode=0x00;
    writeDisplayMode();
}

void DogLcd::blink() {
    blinkMode=0x01;
    writeDisplayMode();
}

void DogLcd::scrollDisplayLeft(void) {
    setInstructionSet(0);
    writeCommand(0x18,30);
}

void DogLcd::scrollDisplayRight(void) {
    setInstructionSet(0);
    writeCommand(0x1C,30);
}

void DogLcd::leftToRight(void) {
    entryMode|=0x02;
    writeCommand(entryMode,30);
}

void DogLcd::rightToLeft(void) {
    entryMode&=~0x02;
    writeCommand(entryMode,30);
}

void DogLcd::autoscroll(void) {
    entryMode|=0x01;
    writeCommand(entryMode,30);
}

void DogLcd::noAutoscroll(void) {
    entryMode&=~0x01;
    writeCommand(entryMode,30);
}

 void DogLcd::createChar(int charPos, uint8_t charMap[]) {
     int baseAddress;
     if(charPos<0 || charPos>7)
	 return;
     baseAddress=charPos*8;
     //changing CGRAM address belongs to different instruction set
	 setInstructionSet(0);
     for (int i=0; i<8; i++) {
	 writeCommand((0x40|(baseAddress+i)),30);
	 writeChar(charMap[i]);
     }
     setInstructionSet(0);
     writeDisplayMode();
}

void DogLcd::writeDisplayMode() {
    writeCommand((0x08 | displayMode | cursorMode | blinkMode),30);
}


void DogLcd::setBacklight(int value, bool usePWM) {
    if(backLight!=-1 && value>=0) {
	if(!usePWM) {
	    if(value==LOW) {
		digitalWrite(backLight,LOW);
	    }
	    else {	    
		digitalWrite(backLight,HIGH);
	    }
	}
	else {	    
	    if(value>255)
		value=255;
	    analogWrite(backLight,value);
	}
    }
}

void DogLcd::setInstructionSet(int is) {
    if(is<0 || is>3)
	return;
    int cmd=instructionSetTemplate | is;
    writeCommand(cmd,30);
}

void DogLcd::writeChar(int value) {
    digitalWrite(lcdRS,HIGH);
    spiTransfer(value,30);
}

void DogLcd::writeCommand(int value,int executionTime) {
    digitalWrite(lcdRS,LOW);
    spiTransfer(value,executionTime);
}

void DogLcd::spiTransfer(int value, int executionTime) {
    digitalWrite(lcdCLK,HIGH);
    digitalWrite(lcdCSB,LOW);
    for(int i=7;i>=0;i--) {
	if(bitRead(value,i)) {
	    digitalWrite(lcdSI,HIGH);
	}
	else {
	    digitalWrite(lcdSI,LOW);
	}
	digitalWrite(lcdCLK,LOW);
	digitalWrite(lcdCLK,HIGH);
    }
    digitalWrite(lcdCSB,HIGH);
    delayMicroseconds(executionTime);
}

