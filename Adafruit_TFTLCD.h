// IMPORTANT: SEE COMMENTS @ LINE 15 REGARDING SHIELD VS BREAKOUT BOARD USAGE.

// Graphics library by ladyada/adafruit with init code from Rossum
// MIT license

#ifndef _ADAFRUIT_TFTLCD_H_
#define _ADAFRUIT_TFTLCD_H_

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define TFTWIDTH   240
#define TFTHEIGHT  320

#include "Adafruit_GFX.h"
#include "lcdDriver2.h"

class Adafruit_TFTLCD: public Adafruit_GFX, public LcdDriver {

public:

	Adafruit_TFTLCD(uint8_t cs, uint8_t cd, uint8_t wr, uint8_t rd, uint8_t rst);
	Adafruit_TFTLCD(void);

	void goTo(int x, int y);
	void drawFastLine(uint16_t x, uint16_t y, uint16_t length,
		                          uint16_t color, uint8_t rotflag);

	void begin(uint16_t id = 0x9325);
	void drawPixel(int16_t x, int16_t y, uint16_t color);
	void drawFastHLine(int16_t x0, int16_t y0, int16_t w, uint16_t color);
	void drawFastVLine(int16_t x0, int16_t y0, int16_t h, uint16_t color);
	void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t c);
	void fillScreen(uint16_t color);

	void setRegisters8(uint8_t *ptr, uint8_t n);
	void setRegisters16(uint16_t *ptr, uint8_t n);
	void setRotation(uint8_t x);
	void setAddrWindow(int x1, int y1, int x2, int y2);
	void flood(uint16_t color, uint32_t len);

	uint16_t readPixel(int16_t x, int16_t y);
	uint16_t readID(void);
	void setLR(void);
	void init();
	void reset();
	uint16_t color565(uint8_t r, uint8_t g, uint8_t b);
};

// For compatibility with sketches written for older versions of library.
// Color function name was changed to 'color565' for parity with 2.2" LCD
// library.
#define Color565 color565

#endif //_ADAFRUIT_TFTLCD_H_
