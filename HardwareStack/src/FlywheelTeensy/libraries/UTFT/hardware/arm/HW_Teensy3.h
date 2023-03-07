/**
 * Set your pins here
 * For 8-bit data bus, pins DB_8 to DB_15 are used.
 *
 * Teensy 3.0 pin definitions created by Dawnmist
 * http://forum.pjrc.com/threads/18002-Teensy-3-0-driving-an-SSD1289-with-utft?p=34719&viewfull=1#post34719
 *
 * There's 3 alternative sets of pin definitions that can be used with the
 * Teensy 3 using this header file. Which one is used is controlled by setting
 * the PORTS define to one of "USE_C_D_PORTS", "USE_B_D_PORTS" or
 * "USE_USER_PORTS".
 * 
 * For LCDs with 16 bit data busses:
 *
 * If PORTS = USE_B_D_PORTS:
 *   - Port B 0-3,16-19 and Port D 0-7 will be used. This is slightly slower
 *     than USE_C_D_PORTS, because we need to do the write to port B in 2 steps
 *     (clear pins 0-3,16-19, then set those pins to match our data). Port D
 *     access is still a single write. This means it takes 3 writes to the pins
 *     instead of the 2 used in USE_C_D_PORTS - but it leaves the SPI port pins
 *     available for use, so gives the fastest result when you need to use an
 *     SD Card/something on the SPI bus as well.
 *     Port D (0-7) = DB_8=2, DB_9=14, DB_10=7, DB_11=8, DB_12=6, DB_13=20, DB_14=21, DB_15=5
 *     Port B (0-3,16-19) = DB_0=16, DB_1=17, DB_2=19, DB_3=18, DB_4=0, DB_5=1, DB_6=32, DB_7=25
 *
 * If PORTS = USE_C_D_PORTS:
 *   - Port C 0-7 and Port D 0-7 will be used. Because these are the bottom
 *     8 pins of each port, we can write to them in one write cycle each. This
 *     is the fastest screen data write - but Port C's pins conflict with the
 *     SPI port, so you won't have access to hardware SPI. If you're using an
 *     SD Card, you will *need* the hardware SPI. This setting should only be
 *     used when hardware SPI is *not* needed.
 *     Port D (0-7) = DB_8=2, DB_9=14, DB_10=7, DB_11=8, DB_12=6, DB_13=20, DB_14=21, DB_15=5
 *     Port C (0-7) = DB_0=15, DB_1=22, DB_2=23, DB_3=9, DB_4=10, DB_5=13, DB_6=11, DB_7=12
 *
 * If PORTS = USE_USER_PORTS
 *   - This allows you to define your own set of pins to use. Since we're not
 *     collating them into groupings that can have their writes optimized, it
 *     will take 16 write cycles to write out the data for one pixel. However,
 *     it also gives you the most flexibility in which pins you use for the
 *     LCD data bus. If high-speed/optimized data writes is not important to you
 *     but simplicity of wiring layout is (or if the above options conflict with
 *     something else you're using on the teensy), you can use this setting to
 *     set the pins where-ever you'd like them to be. By default it's set to
 *     pins DB0-DB15 = 0-15. You should modify the "DB_0" through to "DB_15"
 *     defines to match the pins you wish to use.
 * 
 * For LCDs with 8 bit data busses:
 * Use the pin defines for DB_8 to DB_15 to connect to your LCD. This puts them
 * on Port D in both the USE_C_D_PORTS and USE_B_D_PORTS settings, making those
 * two settings equivalent (and a single write to set the pins on the bus).
 */
#define USE_USER_PORTS 0
#define USE_C_D_PORTS 1
#define USE_B_D_PORTS 2

// SET WHICH PIN DEFINITIONS TO USE HERE
// (only uncomment 1 of these 3 lines)
//
#define PORTS  USE_B_D_PORTS
//#define PORTS  USE_C_D_PORTS
//#define PORTS  USE_USER_PORTS

#if (PORTS == USE_USER_PORTS)
	#pragma message("Using user-defined pins")
	#define DB_0 0
	#define DB_1 1
	#define DB_2 2
	#define DB_3 3
	#define DB_4 4
	#define DB_5 5
	#define DB_6 6
	#define DB_7 7
	#define DB_8 8
	#define DB_9 9
	#define DB_10 10
	#define DB_11 11
	#define DB_12 12
	#define DB_13 13
	#define DB_14 14
	#define DB_15 15
#elif (PORTS == USE_C_D_PORTS)
	#pragma message("Using Ports C&D - pins should be connected to:")
	#pragma message ("DB_0=15, DB_1=22, DB_2=23, DB_3=9, DB_4=10, DB_5=13, DB_6=11, DB_7=12")
	#pragma message ("DB_8=2, DB_9=14, DB_10=7, DB_11=8, DB_12=6, DB_13=20, DB_14=21, DB_15=5")
	#pragma message ("If using a display with an 8-bit bus, use connections to DB_8 to DB_15")
	#define DB_0 15
	#define DB_1 22
	#define DB_2 23
	#define DB_3 9
	#define DB_4 10
	#define DB_5 13
	#define DB_6 11
	#define DB_7 12
	#define DB_8 2
	#define DB_9 14
	#define DB_10 7
	#define DB_11 8
	#define DB_12 6
	#define DB_13 20
	#define DB_14 21
	#define DB_15 5
#elif (PORTS == USE_B_D_PORTS)
	#pragma message("Using Ports B&D - pins should be connected to:")
	#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
	#pragma message ("DB_0=16, DB_1=17, DB_2=19, DB_3=18, DB_4=0, DB_5=1, DB_6=29, DB_7=30")
	#else
	#pragma message ("DB_0=16, DB_1=17, DB_2=19, DB_3=18, DB_4=0, DB_5=1, DB_6=32, DB_7=25")
	#endif
	#pragma message ("DB_8=2, DB_9=14, DB_10=7, DB_11=8, DB_12=6, DB_13=20, DB_14=21, DB_15=5")
	#pragma message ("If using a display with an 8-bit bus, use connections to DB_8 to DB_15")
	#define DB_0 16
	#define DB_1 17
	#define DB_2 19
	#define DB_3 18
	#define DB_4 0
	#define DB_5 1
	#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
	#define DB_6 29
	#define DB_7 30
	#else
	#define DB_6 32
	#define DB_7 25
	#endif
	#define DB_8 2
	#define DB_9 14
	#define DB_10 7
	#define DB_11 8
	#define DB_12 6
	#define DB_13 20
	#define DB_14 21
	#define DB_15 5
#endif

// *** Hardware specific functions ***
void UTFT::_hw_special_init()
{
}

void UTFT::LCD_Writ_Bus(char ch,char cl, byte mode)
{
	switch (mode)
	{
	case 1:
		// Not implemented
		break;
	case 8:
#if (PORTS == USE_C_D_PORTS) || (PORTS == USE_B_D_PORTS)
		*(volatile uint8_t *)(&GPIOD_PDOR) = ch;
		pulse_low(P_WR, B_WR);
		*(volatile uint8_t *)(&GPIOD_PDOR) = cl;
		pulse_low(P_WR, B_WR);
#else
		((ch & 0x80) != 0) ? digitalWriteFast(DB_15, HIGH) : digitalWriteFast(DB_15, LOW);
		((ch & 0x40) != 0) ? digitalWriteFast(DB_14, HIGH) : digitalWriteFast(DB_14, LOW);
		((ch & 0x20) != 0) ? digitalWriteFast(DB_13, HIGH) : digitalWriteFast(DB_13, LOW);
		((ch & 0x10) != 0) ? digitalWriteFast(DB_12, HIGH) : digitalWriteFast(DB_12, LOW);
		((ch & 0x08) != 0) ? digitalWriteFast(DB_11, HIGH) : digitalWriteFast(DB_11, LOW);
		((ch & 0x04) != 0) ? digitalWriteFast(DB_10, HIGH) : digitalWriteFast(DB_10, LOW);
		((ch & 0x02) != 0) ? digitalWriteFast(DB_9, HIGH) : digitalWriteFast(DB_9, LOW);
		((ch & 0x01) != 0) ? digitalWriteFast(DB_8, HIGH) : digitalWriteFast(DB_8, LOW);
		pulse_low(P_WR, B_WR);
		((cl & 0x80) != 0) ? digitalWriteFast(DB_15, HIGH) : digitalWriteFast(DB_15, LOW);
		((cl & 0x40) != 0) ? digitalWriteFast(DB_14, HIGH) : digitalWriteFast(DB_14, LOW);
		((cl & 0x20) != 0) ? digitalWriteFast(DB_13, HIGH) : digitalWriteFast(DB_13, LOW);
		((cl & 0x10) != 0) ? digitalWriteFast(DB_12, HIGH) : digitalWriteFast(DB_12, LOW);
		((cl & 0x08) != 0) ? digitalWriteFast(DB_11, HIGH) : digitalWriteFast(DB_11, LOW);
		((cl & 0x04) != 0) ? digitalWriteFast(DB_10, HIGH) : digitalWriteFast(DB_10, LOW);
		((cl & 0x02) != 0) ? digitalWriteFast(DB_9, HIGH) : digitalWriteFast(DB_9, LOW);
		((cl & 0x01) != 0) ? digitalWriteFast(DB_8, HIGH) : digitalWriteFast(DB_8, LOW);
		pulse_low(P_WR, B_WR);
#endif
		break;
	case 16:
#if (PORTS == USE_C_D_PORTS)
		*(volatile uint8_t *)(&GPIOD_PDOR) = ch;
		*(volatile uint8_t *)(&GPIOC_PDOR) = cl;
#elif (PORTS == USE_B_D_PORTS)
		*(volatile uint8_t *)(&GPIOD_PDOR) = ch;
		// clear data lines B0-3,B16-19
  		GPIOB_PCOR = 0x000F000F;
  		// set data lines 0-3,16-19 if set in cl
        GPIOB_PSOR = (0x0F & cl) | ((cl >> 4) << 16);
#else
		((ch & 0x80) != 0) ? digitalWriteFast(DB_15, HIGH) : digitalWriteFast(DB_15, LOW);
		((ch & 0x40) != 0) ? digitalWriteFast(DB_14, HIGH) : digitalWriteFast(DB_14, LOW);
		((ch & 0x20) != 0) ? digitalWriteFast(DB_13, HIGH) : digitalWriteFast(DB_13, LOW);
		((ch & 0x10) != 0) ? digitalWriteFast(DB_12, HIGH) : digitalWriteFast(DB_12, LOW);
		((ch & 0x08) != 0) ? digitalWriteFast(DB_11, HIGH) : digitalWriteFast(DB_11, LOW);
		((ch & 0x04) != 0) ? digitalWriteFast(DB_10, HIGH) : digitalWriteFast(DB_10, LOW);
		((ch & 0x02) != 0) ? digitalWriteFast(DB_9, HIGH) : digitalWriteFast(DB_9, LOW);
		((ch & 0x01) != 0) ? digitalWriteFast(DB_8, HIGH) : digitalWriteFast(DB_8, LOW);
		((cl & 0x80) != 0) ? digitalWriteFast(DB_7, HIGH) : digitalWriteFast(DB_7, LOW);
		((cl & 0x40) != 0) ? digitalWriteFast(DB_6, HIGH) : digitalWriteFast(DB_6, LOW);
		((cl & 0x20) != 0) ? digitalWriteFast(DB_5, HIGH) : digitalWriteFast(DB_5, LOW);
		((cl & 0x10) != 0) ? digitalWriteFast(DB_4, HIGH) : digitalWriteFast(DB_4, LOW);
		((cl & 0x08) != 0) ? digitalWriteFast(DB_3, HIGH) : digitalWriteFast(DB_3, LOW);
		((cl & 0x04) != 0) ? digitalWriteFast(DB_2, HIGH) : digitalWriteFast(DB_2, LOW);
		((cl & 0x02) != 0) ? digitalWriteFast(DB_1, HIGH) : digitalWriteFast(DB_1, LOW);
		((cl & 0x01) != 0) ? digitalWriteFast(DB_0, HIGH) : digitalWriteFast(DB_0, LOW);
#endif
		pulse_low(P_WR, B_WR);
		break;
	}
}

void UTFT::_set_direction_registers(byte mode)
{
    pinMode(DB_8, OUTPUT);
    pinMode(DB_9, OUTPUT);
    pinMode(DB_10, OUTPUT);
    pinMode(DB_11, OUTPUT);
    pinMode(DB_12, OUTPUT);
    pinMode(DB_13, OUTPUT);
    pinMode(DB_14, OUTPUT);
    pinMode(DB_15, OUTPUT);
    if (mode == 16)
    {
	pinMode(DB_0, OUTPUT);
	pinMode(DB_1, OUTPUT);
	pinMode(DB_2, OUTPUT);
	pinMode(DB_3, OUTPUT);
	pinMode(DB_4, OUTPUT);
	pinMode(DB_5, OUTPUT);
	pinMode(DB_6, OUTPUT);
	pinMode(DB_7, OUTPUT);
    }
}
void UTFT::_fast_fill_16(int ch, int cl, long pix)
{
	long blocks;
#if (PORTS == USE_C_D_PORTS)
		*(volatile uint8_t *)(&GPIOD_PDOR) = ch;
		*(volatile uint8_t *)(&GPIOC_PDOR) = cl;
#elif (PORTS == USE_B_D_PORTS)
		*(volatile uint8_t *)(&GPIOD_PDOR) = ch;
		// clear data lines B0-3,B16-19
  		GPIOB_PCOR = 0x000F000F;
  		// set data lines 0-3,16-19 if set in cl
        GPIOB_PSOR = (0x0F & cl) | ((cl >> 4) << 16);
#else
		((ch & 0x80) != 0) ? digitalWriteFast(DB_15, HIGH) : digitalWriteFast(DB_15, LOW);
		((ch & 0x40) != 0) ? digitalWriteFast(DB_14, HIGH) : digitalWriteFast(DB_14, LOW);
		((ch & 0x20) != 0) ? digitalWriteFast(DB_13, HIGH) : digitalWriteFast(DB_13, LOW);
		((ch & 0x10) != 0) ? digitalWriteFast(DB_12, HIGH) : digitalWriteFast(DB_12, LOW);
		((ch & 0x08) != 0) ? digitalWriteFast(DB_11, HIGH) : digitalWriteFast(DB_11, LOW);
		((ch & 0x04) != 0) ? digitalWriteFast(DB_10, HIGH) : digitalWriteFast(DB_10, LOW);
		((ch & 0x02) != 0) ? digitalWriteFast(DB_9, HIGH) : digitalWriteFast(DB_9, LOW);
		((ch & 0x01) != 0) ? digitalWriteFast(DB_8, HIGH) : digitalWriteFast(DB_8, LOW);
		((cl & 0x80) != 0) ? digitalWriteFast(DB_7, HIGH) : digitalWriteFast(DB_7, LOW);
		((cl & 0x40) != 0) ? digitalWriteFast(DB_6, HIGH) : digitalWriteFast(DB_6, LOW);
		((cl & 0x20) != 0) ? digitalWriteFast(DB_5, HIGH) : digitalWriteFast(DB_5, LOW);
		((cl & 0x10) != 0) ? digitalWriteFast(DB_4, HIGH) : digitalWriteFast(DB_4, LOW);
		((cl & 0x08) != 0) ? digitalWriteFast(DB_3, HIGH) : digitalWriteFast(DB_3, LOW);
		((cl & 0x04) != 0) ? digitalWriteFast(DB_2, HIGH) : digitalWriteFast(DB_2, LOW);
		((cl & 0x02) != 0) ? digitalWriteFast(DB_1, HIGH) : digitalWriteFast(DB_1, LOW);
		((cl & 0x01) != 0) ? digitalWriteFast(DB_0, HIGH) : digitalWriteFast(DB_0, LOW);
#endif
	blocks = pix/16;
	for (int i=0; i<blocks; i++)
	{
		pulse_low(P_WR, B_WR);
		pulse_low(P_WR, B_WR);
		pulse_low(P_WR, B_WR);
		pulse_low(P_WR, B_WR);
		pulse_low(P_WR, B_WR);
		pulse_low(P_WR, B_WR);
		pulse_low(P_WR, B_WR);
		pulse_low(P_WR, B_WR);
		pulse_low(P_WR, B_WR);
		pulse_low(P_WR, B_WR);
		pulse_low(P_WR, B_WR);
		pulse_low(P_WR, B_WR);
		pulse_low(P_WR, B_WR);
		pulse_low(P_WR, B_WR);
		pulse_low(P_WR, B_WR);
		pulse_low(P_WR, B_WR);
	}
	if ((pix % 16) != 0)
		for (int i=0; i<(pix % 16); i++)
		{
			pulse_low(P_WR, B_WR);
		}
}

void UTFT::_fast_fill_8(int ch, long pix)
{
	long blocks;
#if (PORTS == USE_C_D_PORTS) || (PORTS == USE_B_D_PORTS)
		*(volatile uint8_t *)(&GPIOD_PDOR) = ch;
#else
		((ch & 0x80) != 0) ? digitalWriteFast(DB_15, HIGH) : digitalWriteFast(DB_15, LOW);
		((ch & 0x40) != 0) ? digitalWriteFast(DB_14, HIGH) : digitalWriteFast(DB_14, LOW);
		((ch & 0x20) != 0) ? digitalWriteFast(DB_13, HIGH) : digitalWriteFast(DB_13, LOW);
		((ch & 0x10) != 0) ? digitalWriteFast(DB_12, HIGH) : digitalWriteFast(DB_12, LOW);
		((ch & 0x08) != 0) ? digitalWriteFast(DB_11, HIGH) : digitalWriteFast(DB_11, LOW);
		((ch & 0x04) != 0) ? digitalWriteFast(DB_10, HIGH) : digitalWriteFast(DB_10, LOW);
		((ch & 0x02) != 0) ? digitalWriteFast(DB_9, HIGH) : digitalWriteFast(DB_9, LOW);
		((ch & 0x01) != 0) ? digitalWriteFast(DB_8, HIGH) : digitalWriteFast(DB_8, LOW);
#endif

	blocks = pix/16;
	for (int i=0; i<blocks; i++)
	{
		pulse_low(P_WR, B_WR);pulse_low(P_WR, B_WR);
		pulse_low(P_WR, B_WR);pulse_low(P_WR, B_WR);
		pulse_low(P_WR, B_WR);pulse_low(P_WR, B_WR);
		pulse_low(P_WR, B_WR);pulse_low(P_WR, B_WR);
		pulse_low(P_WR, B_WR);pulse_low(P_WR, B_WR);
		pulse_low(P_WR, B_WR);pulse_low(P_WR, B_WR);
		pulse_low(P_WR, B_WR);pulse_low(P_WR, B_WR);
		pulse_low(P_WR, B_WR);pulse_low(P_WR, B_WR);
		pulse_low(P_WR, B_WR);pulse_low(P_WR, B_WR);
		pulse_low(P_WR, B_WR);pulse_low(P_WR, B_WR);
		pulse_low(P_WR, B_WR);pulse_low(P_WR, B_WR);
		pulse_low(P_WR, B_WR);pulse_low(P_WR, B_WR);
		pulse_low(P_WR, B_WR);pulse_low(P_WR, B_WR);
		pulse_low(P_WR, B_WR);pulse_low(P_WR, B_WR);
		pulse_low(P_WR, B_WR);pulse_low(P_WR, B_WR);
		pulse_low(P_WR, B_WR);pulse_low(P_WR, B_WR);
	}
	if ((pix % 16) != 0)
		for (int i=0; i<(pix % 16); i++)
		{
			pulse_low(P_WR, B_WR);pulse_low(P_WR, B_WR);
		}
}
