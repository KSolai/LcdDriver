// IMPORTANT: LIBRARY MUST BE SPECIFICALLY CONFIGURED FOR EITHER TFT SHIELD
// OR BREAKOUT BOARD USAGE.  SEE RELEVANT COMMENTS IN Adafruit_TFTLCD.h

// Graphics library by ladyada/adafruit with init code from Rossum
// MIT license

#include "Adafruit_TFTLCD.h"
#include "registers.h"

// Constructor for break-out board (configurable LCD control lines).
// Can still use this w/shield, but parameters are ignored.
Adafruit_TFTLCD::Adafruit_TFTLCD(uint8_t cs, uint8_t cd, uint8_t wr, uint8_t rd, uint8_t reset) :
		Adafruit_GFX(TFTWIDTH, TFTHEIGHT), LcdDriver(cs, cd, wr, rd, reset)
{
	init();
}

// Constructor for shield (fixed LCD control lines)
Adafruit_TFTLCD::Adafruit_TFTLCD(void) :
		Adafruit_GFX(TFTWIDTH, TFTHEIGHT)
{
	init();
}

// Initialization common to both shield & break-out configurations
void Adafruit_TFTLCD::init(void)
{
	rotation = 0;
	cursor_y = cursor_x = 0;
	textsize = 1;
	textcolor = 0xFFFF;
	_width = TFTWIDTH;
	_height = TFTHEIGHT;
}
//-------------------------------------------------------------------------------------------------
// Initialization command tables for different LCD controllers
//********************** DISPLAY TYPE ONE *************************************
#define TFTLCD_START_OSC          0x00
#define TFTLCD_DRIV_OUT_CTRL      0x01
#define TFTLCD_DRIV_WAV_CTRL      0x02
#define TFTLCD_ENTRY_MOD          0x03
#define TFTLCD_RESIZE_CTRL        0x04
#define TFTLCD_DISP_CTRL1         0x07
#define TFTLCD_DISP_CTRL2         0x08
#define TFTLCD_DISP_CTRL3         0x09
#define TFTLCD_DISP_CTRL4         0x0A
#define TFTLCD_RGB_DISP_IF_CTRL1  0x0C
#define TFTLCD_FRM_MARKER_POS     0x0D
#define TFTLCD_RGB_DISP_IF_CTRL2  0x0F
#define TFTLCD_POW_CTRL1          0x10
#define TFTLCD_POW_CTRL2          0x11
#define TFTLCD_POW_CTRL3          0x12
#define TFTLCD_POW_CTRL4          0x13
#define TFTLCD_GRAM_HOR_AD        0x20
#define TFTLCD_GRAM_VER_AD        0x21
#define TFTLCD_RW_GRAM            0x22
#define TFTLCD_POW_CTRL7          0x29
#define TFTLCD_FRM_RATE_COL_CTRL  0x2B
#define TFTLCD_GAMMA_CTRL1        0x30
#define TFTLCD_GAMMA_CTRL2        0x31
#define TFTLCD_GAMMA_CTRL3        0x32
#define TFTLCD_GAMMA_CTRL4        0x35
#define TFTLCD_GAMMA_CTRL5        0x36
#define TFTLCD_GAMMA_CTRL6        0x37
#define TFTLCD_GAMMA_CTRL7        0x38
#define TFTLCD_GAMMA_CTRL8        0x39
#define TFTLCD_GAMMA_CTRL9        0x3C
#define TFTLCD_GAMMA_CTRL10       0x3D
#define TFTLCD_HOR_START_AD       0x50
#define TFTLCD_HOR_END_AD         0x51
#define TFTLCD_VER_START_AD       0x52
#define TFTLCD_VER_END_AD         0x53
#define TFTLCD_GATE_SCAN_CTRL1    0x60
#define TFTLCD_GATE_SCAN_CTRL2    0x61
#define TFTLCD_GATE_SCAN_CTRL3    0x6A
#define TFTLCD_PART_IMG1_DISP_POS 0x80
#define TFTLCD_PART_IMG1_START_AD 0x81
#define TFTLCD_PART_IMG1_END_AD   0x82
#define TFTLCD_PART_IMG2_DISP_POS 0x83
#define TFTLCD_PART_IMG2_START_AD 0x84
#define TFTLCD_PART_IMG2_END_AD   0x85
#define TFTLCD_PANEL_IF_CTRL1     0x90
#define TFTLCD_PANEL_IF_CTRL2     0x92
#define TFTLCD_PANEL_IF_CTRL3     0x93
#define TFTLCD_PANEL_IF_CTRL4     0x95
#define TFTLCD_PANEL_IF_CTRL5     0x97
#define TFTLCD_PANEL_IF_CTRL6     0x98
#define TFTLCD_DELAYCMD		0xFF

//------------------------------------------------------------------------------------------------
static const uint16_t RegValues[] PROGMEM =
{
	TFTLCD_START_OSC, 			0x0001,
	TFTLCD_DRIV_OUT_CTRL, 		0x0100,
	TFTLCD_DRIV_WAV_CTRL, 		0x0700,
	TFTLCD_ENTRY_MOD, 			0x1030,

	TFTLCD_RESIZE_CTRL, 		0x0000,
	TFTLCD_DISP_CTRL2, 			0x0202,
	TFTLCD_DISP_CTRL3, 			0x0000,
	TFTLCD_DISP_CTRL4, 			0x0000,

	TFTLCD_RGB_DISP_IF_CTRL1, 	0x0000,
	TFTLCD_FRM_MARKER_POS, 		0x0000,
	TFTLCD_RGB_DISP_IF_CTRL2, 	0x0000,

	//*******POWER CONTROL REGISTER INITIAL*******//
	TFTLCD_POW_CTRL1, 			0x0000,
	TFTLCD_POW_CTRL2, 			0x0000,
	TFTLCD_POW_CTRL3, 			0x0000,
	TFTLCD_POW_CTRL4, 			0x0000,	TFTLCD_DELAYCMD, 200,
	TFTLCD_POW_CTRL1, 			0x17B0,
	TFTLCD_POW_CTRL2, 			0x0037,	TFTLCD_DELAYCMD, 100,
	TFTLCD_POW_CTRL3, 			0x0138,
	TFTLCD_POW_CTRL4, 			0x1700,
	TFTLCD_POW_CTRL7, 			0x000D,	TFTLCD_DELAYCMD, 100,

	TFTLCD_GRAM_HOR_AD, 		0x0000,
	TFTLCD_GRAM_VER_AD, 		0x0000,
	//******GAMMA CLUSTER SETTING******//
	TFTLCD_GAMMA_CTRL1, 		0x0001,
	TFTLCD_GAMMA_CTRL2, 		0x0606,
	TFTLCD_GAMMA_CTRL3, 		0x0304,
	TFTLCD_GAMMA_CTRL4, 		0x0103,
	TFTLCD_GAMMA_CTRL5, 		0x011D,
	TFTLCD_GAMMA_CTRL6,			0x0404,
	TFTLCD_GAMMA_CTRL7, 		0x0404,
	TFTLCD_GAMMA_CTRL8, 		0x0404,
	TFTLCD_GAMMA_CTRL9, 		0x0700,
	TFTLCD_GAMMA_CTRL10, 		0x0A1F,

	// -----------DISPLAY WINDOWS 240*320-------------//
	TFTLCD_HOR_START_AD, 		0,
	TFTLCD_HOR_END_AD, 			TFTWIDTH-1,
	TFTLCD_VER_START_AD, 		0,
	TFTLCD_VER_END_AD, 			TFTHEIGHT-1,

	//-----FRAME RATE SETTING-------//
	TFTLCD_GATE_SCAN_CTRL1, 	0x2700,
	TFTLCD_GATE_SCAN_CTRL2, 	0x0001,
	TFTLCD_GATE_SCAN_CTRL3, 	0x0000,

	//------------ RTNI SETTING
	TFTLCD_PANEL_IF_CTRL1, 		0x0010,
	TFTLCD_PANEL_IF_CTRL2, 		0x0000,
	TFTLCD_PANEL_IF_CTRL3, 		0x0003,
	TFTLCD_PANEL_IF_CTRL4, 		0x0101,
	TFTLCD_PANEL_IF_CTRL5, 		0x0000,
	TFTLCD_PANEL_IF_CTRL6, 		0x0000,

	//-------DISPLAY ON------//
	TFTLCD_DISP_CTRL1, 			0x0021,
	TFTLCD_DISP_CTRL1, 			0x0031,
	TFTLCD_DISP_CTRL1, 			0x0173
};

#define TFTLCD_DELAY 0xFF

//------------------------------------------------------------------------------------------------
static const uint8_t HX8347G_regValues[] PROGMEM =
{
	0x2E, 0x89, 0x29, 0x8F, 0x2B, 0x02, 0xE2, 0x00, 0xE4, 0x01, 0xE5, 0x10, 0xE6, 0x01, 0xE7, 0x10,
	0xE8, 0x70, 0xF2, 0x00, 0xEA, 0x00, 0xEB, 0x20, 0xEC, 0x3C, 0xED, 0xC8, 0xE9, 0x38, 0xF1,
	0x01,
	// skip gamma, do later
	0x1B, 0x1A, 0x1A, 0x02, 0x24, 0x61, 0x25, 0x5C,
	0x18, 0x36, 0x19, 0x01, 0x1F, 0x88,
	TFTLCD_DELAY,
	5, // delay 5 ms
	0x1F, 0x80,
	TFTLCD_DELAY, 5, 0x1F, 0x90,
	TFTLCD_DELAY, 5, 0x1F, 0xD4,
	TFTLCD_DELAY, 5, 0x17, 0x05,

	0x36, 0x09, 0x28, 0x38,
	TFTLCD_DELAY, 40, 0x28, 0x3C,

	0x02, 0x00, 0x03, 0x00, 0x04, 0x00, 0x05, 0xEF, 0x06, 0x00, 0x07, 0x00, 0x08, 0x01, 0x09,
	0x3F
};

//------------------------------------------------------------------------------------------------
static const uint8_t HX8357D_regValues[] PROGMEM =
{
	HX8357_SWRESET, 0,
	HX8357D_SETC, 3, 0xFF, 0x83, 0x57,
	TFTLCD_DELAY, 250,
	HX8357_SETRGB, 4, 0x00, 0x00, 0x06, 0x06,
	HX8357D_SETCOM, 1, 0x25,  // -1.52V
	HX8357_SETOSC, 1, 0x68,  // Normal mode 70Hz, Idle mode 55 Hz
	HX8357_SETPANEL, 1, 0x05,  // BGR, Gate direction swapped
	HX8357_SETPWR1, 6, 0x00, 0x15, 0x1C, 0x1C, 0x83, 0xAA,
	HX8357D_SETSTBA, 6, 0x50, 0x50, 0x01, 0x3C, 0x1E, 0x08,
	// MEME GAMMA HERE
	HX8357D_SETCYC, 7, 0x02, 0x40, 0x00, 0x2A, 0x2A, 0x0D, 0x78,
	HX8357_COLMOD, 1, 0x55,
	HX8357_MADCTL, 1, 0xC0,
	HX8357_TEON, 1, 0x00,
	HX8357_TEARLINE, 2, 0x00, 0x02,
	HX8357_SLPOUT, 0,
	TFTLCD_DELAY, 150,
	HX8357_DISPON, 0,
	TFTLCD_DELAY, 50,
};

//------------------------------------------------------------------------------------------------
static const uint16_t ILI932x_regValues[] PROGMEM =
{
	ILI932X_START_OSC, 0x0001, 			// Start oscillator
	TFTLCD_DELAY, 50,     				// 50 millisecond delay
	ILI932X_DRIV_OUT_CTRL, 0x0100,
	ILI932X_DRIV_WAV_CTRL, 0x0700,
	ILI932X_ENTRY_MOD, 0x1030,
	ILI932X_RESIZE_CTRL, 0x0000,
	ILI932X_DISP_CTRL2, 0x0202,
	ILI932X_DISP_CTRL3, 0x0000,
	ILI932X_DISP_CTRL4, 0x0000,
	ILI932X_RGB_DISP_IF_CTRL1, 0x0,
	ILI932X_FRM_MARKER_POS, 0x0,
	ILI932X_RGB_DISP_IF_CTRL2, 0x0,
	ILI932X_POW_CTRL1, 0x0000,
	ILI932X_POW_CTRL2, 0x0007,
	ILI932X_POW_CTRL3, 0x0000,
	ILI932X_POW_CTRL4, 0x0000,
	TFTLCD_DELAY, 200,
	ILI932X_POW_CTRL1, 0x1690,
	ILI932X_POW_CTRL2, 0x0227,
	TFTLCD_DELAY, 50,
	ILI932X_POW_CTRL3, 0x001A,
	TFTLCD_DELAY, 50,
	ILI932X_POW_CTRL4, 0x1800,
	ILI932X_POW_CTRL7, 0x002A,
	TFTLCD_DELAY, 50,
	ILI932X_GAMMA_CTRL1, 0x0000,
	ILI932X_GAMMA_CTRL2, 0x0000,
	ILI932X_GAMMA_CTRL3, 0x0000,
	ILI932X_GAMMA_CTRL4, 0x0206,
	ILI932X_GAMMA_CTRL5, 0x0808,
	ILI932X_GAMMA_CTRL6, 0x0007,
	ILI932X_GAMMA_CTRL7, 0x0201,
	ILI932X_GAMMA_CTRL8, 0x0000,
	ILI932X_GAMMA_CTRL9, 0x0000,
	ILI932X_GAMMA_CTRL10, 0x0000,
	ILI932X_GRAM_HOR_AD, 0x0000,
	ILI932X_GRAM_VER_AD, 0x0000,
	ILI932X_HOR_START_AD, 0x0000,
	ILI932X_HOR_END_AD, 0x00EF,
	ILI932X_VER_START_AD, 0X0000,
	ILI932X_VER_END_AD, 0x013F,
	ILI932X_GATE_SCAN_CTRL1, 0xA700, // Driver Output Control (R60h)
	ILI932X_GATE_SCAN_CTRL2, 0x0003, // Driver Output Control (R61h)
	ILI932X_GATE_SCAN_CTRL3, 0x0000, // Driver Output Control (R62h)
	ILI932X_PANEL_IF_CTRL1, 0X0010, // Panel Interface Control 1 (R90h)
	ILI932X_PANEL_IF_CTRL2, 0X0000,
	ILI932X_PANEL_IF_CTRL3, 0X0003,
	ILI932X_PANEL_IF_CTRL4, 0X1100,
	ILI932X_PANEL_IF_CTRL5, 0X0000,
	ILI932X_PANEL_IF_CTRL6, 0X0000,
	ILI932X_DISP_CTRL1, 0x0133, // Main screen turn on
};
//------------------------------------------------------------------------------------------------
static const uint16_t LGDP4535_regValues[] PROGMEM =
{
	//LGDP4535
	0x15, 0x0030, 0x9A, 0x0010, 0x11, 0x0020, 0x10, 0x3428, 0x12, 0x0002, 0x13, 0x1046,
	TFTLCD_DELAY,
	40, //40ms
	0x12, 0x0012,
	TFTLCD_DELAY,
	40, //40ms
	0x10, 0x3420, 0x13, 0x3046,
	TFTLCD_DELAY,
	70, //70ms

	/******gamma setting******/
	0x30, 0x0000, 0x31, 0x0402, 0x32, 0x0307, 0x33, 0x0304, 0x34, 0x0004, 0x35, 0x0401, 0x36,
	0x0707, 0x37, 0x0305, 0x38, 0x0610, 0x39, 0x0610,

	/********display mode******/
	0x01, 0x0100, 0x02, 0x0300, 0x03, 0x1030, 0x08, 0x0808, 0x0A, 0x0008, 0x60, 0x2700, 0x61,
	0x0001,

	0x90, 0x013E, 0x92, 0x0100, 0x93, 0x0100, 0xA0, 0x3000, 0xA3, 0x0010,

	/*******display on*******/
	0x07, 0x0001, 0x07, 0x0021, 0x07, 0x0023, 0x07, 0x0033, 0x07, 0x0133,

};
//------------------------------------------------------------------------------------------------
void Adafruit_TFTLCD::begin(uint16_t id)
{
	uint8_t i = 0;
	reset();

	delay(200);

	if ((id == 0x9325) || (id == 0x9328))
	{
		uint16_t a, d;
		driver = ID_932X;
		while (i < sizeof(ILI932x_regValues) / sizeof(uint16_t))
		{
			a = pgm_read_word(&ILI932x_regValues[i++]);
			d = pgm_read_word(&ILI932x_regValues[i++]);
			if (a == TFTLCD_DELAY)
				delay(d);
			else
				writeRegister16(a, d);
		}
		setRotation(rotation);
		setAddrWindow(0, 0, TFTWIDTH - 1, TFTHEIGHT - 1);
	}
	else if (id == 0x4535)
	{
		uint16_t a, d;
		driver = ID_932X;
		while (i < sizeof(LGDP4535_regValues) / sizeof(uint16_t))
		{
			a = pgm_read_word(&LGDP4535_regValues[i++]);
			d = pgm_read_word(&LGDP4535_regValues[i++]);
			if (a == TFTLCD_DELAY)
				delay(d);
			else
				writeRegister16(a, d);
		}
		setRotation(rotation);
		setAddrWindow(0, 0, TFTWIDTH - 1, TFTHEIGHT - 1);
	}
	else if (id == 0x9341)
	{
		driver = ID_9341;
		writeRegister8(ILI9341_SOFTRESET, 0);
		delay(50);
		writeRegister8(ILI9341_DISPLAYOFF, 0);

		writeRegister8(ILI9341_POWERCONTROL1, 0x23);
		writeRegister8(ILI9341_POWERCONTROL2, 0x10);
		writeRegister16(ILI9341_VCOMCONTROL1, 0x2B2B);
		writeRegister8(ILI9341_VCOMCONTROL2, 0xC0);
		writeRegister8(ILI9341_MEMCONTROL,
		ILI9341_MADCTL_MY | ILI9341_MADCTL_BGR);
		writeRegister8(ILI9341_PIXELFORMAT, 0x55);
		writeRegister16(ILI9341_FRAMECONTROL, 0x001B);

		writeRegister8(ILI9341_ENTRYMODE, 0x07);
		/* writeRegister32(ILI9341_DISPLAYFUNC, 0x0A822700);*/

		writeRegister8(ILI9341_SLEEPOUT, 0);
		delay(150);
		writeRegister8(ILI9341_DISPLAYON, 0);
		delay(500);
		setAddrWindow(0, 0, TFTWIDTH - 1, TFTHEIGHT - 1);
	}
	else if (id == 0x8357)
	{
		driver = ID_HX8357D;
		while (i < sizeof(HX8357D_regValues))
		{
			uint8_t r = pgm_read_byte(&HX8357D_regValues[i++]);
			uint8_t len = pgm_read_byte(&HX8357D_regValues[i++]);
			if (r == TFTLCD_DELAY)
			{
				delay(len);
			}
			else
			{
				writeCommand(r); //write8(r);
				for (uint8_t d = 0; d < len; d++)
				{
					uint8_t x = pgm_read_byte(&HX8357D_regValues[i++]);
					writeData(x); //write8(x);
				}
			}
		}
	}
	else if (id == 0x7575)
	{
		uint8_t a, d;
		driver = ID_7575;
		while (i < sizeof(HX8347G_regValues))
		{
			a = pgm_read_byte(&HX8347G_regValues[i++]);
			d = pgm_read_byte(&HX8347G_regValues[i++]);
			if (a == TFTLCD_DELAY)
				delay(d);
			else
				writeRegister8(a, d);
		}
		setRotation(rotation);
		setLR(); // Lower-right corner of address window

	}
	else if (id == 0xC505)
	{
		driver = ID_C505;
		uint16_t a, d;
		while (i < sizeof(RegValues) / sizeof(uint16_t))
		{
			a = pgm_read_word(&RegValues[i++]);
			d = pgm_read_word(&RegValues[i++]);
			if (a == TFTLCD_DELAY)
				delay(d);
			else
				writeRegister16(a, d);
		}
	}
	else
	{
		driver = ID_UNKNOWN;
		return;
	}
}

//---------------------------------------------------------------------------------------
void Adafruit_TFTLCD::goTo(int x, int y) {
  writeRegister16(TFTLCD_GRAM_HOR_AD, x);   // GRAM Address Set (Horizontal Address) (R20h)
  writeRegister16(TFTLCD_GRAM_VER_AD, y);   // GRAM Address Set (Vertical Address) (R21h)
  writeCommand((uint16_t)TFTLCD_RW_GRAM);   // Write Data to GRAM (R22h)
}

//---------------------------------------------------------------------------------------
void Adafruit_TFTLCD::drawFastLine(uint16_t x, uint16_t y, uint16_t length,
                          uint16_t color, uint8_t rotflag)
{
  uint16_t newentrymod =0;
  switch (rotation) {
    case 0:
      newentrymod = rotflag ?  0x1028 : 0x1030; // vertical or horizontal
      break;
    case 1:
      swap(x, y);
      x = TFTWIDTH - x - 1;
      newentrymod = rotflag ?  0x1000 : 0x1028; // vertical or horizontal
      break;
    case 2:
      x =  TFTWIDTH - x - 1;
      y =  TFTHEIGHT - y - 1;
      newentrymod = rotflag ?  0x1008 : 0x1020; // vertical or horizontal
      break;
    case 3:
      swap(x, y);
      y = TFTHEIGHT - y - 1;
      newentrymod = rotflag ?  0x1030 : 0x1008; // vertical or horizontal
      break;
  }

  writeRegister16(TFTLCD_ENTRY_MOD, newentrymod);
  goTo(x,y);
  setWrDirection(false);
  while (length--) {
    writeData(color);
  }
  resetDirection();
  writeRegister16(TFTLCD_ENTRY_MOD, 0x1028);
}

//---------------------------------------------------------------------------------------
// Sets the LCD address window (and address counter, on 932X).
// Relevant to rect/screen fills and H/V lines.  Input coordinates are
// assumed pre-sorted (e.g. x2 >= x1).
void Adafruit_TFTLCD::setAddrWindow(int x1, int y1, int x2, int y2)
{
	if ((driver == ID_932X)||(driver == ID_C505))
	{
		// Values passed are in current (possibly rotated) coordinate
		// system.  932X requires hardware-native coords regardless of
		// MADCTL, so rotate inputs as needed.  The address counter is
		// set to the top-left corner -- although fill operations can be
		// done in any direction, the current screen rotation is applied
		// because some users find it disconcerting when a fill does not
		// occur top-to-bottom.
		int x, y, t;
		switch (rotation)
		{
		case 1:
			swap(x2,y2);
			swap(x1,y1);
			x1 = TFTWIDTH - 1 - x2;
			x2 = TFTWIDTH - 1 - y1;
			x = x2;
			y = y1;
			break;
		case 2:
			t = x1;
			x1 = TFTWIDTH - 1 - x2;
			x2 = TFTWIDTH - 1 - t;
			t = y1;
			y1 = TFTHEIGHT - 1 - y2;
			y2 = TFTHEIGHT - 1 - t;
			x = x2;
			y = y2;
			break;
		case 3:
			swap(x2,y2);
			swap(x1,y1);
			y1 = TFTHEIGHT - 1 - y2;
			y2 = TFTHEIGHT - 1 - x1;
			x = x1;
			y = y1;
			break;
		case 0:
		default:
			x = x1;
			y = y1;
			break;
		}
		writeRegister16(TFTLCD_HOR_START_AD, x1); // Set address window
		writeRegister16(TFTLCD_HOR_END_AD, x2);
		writeRegister16(TFTLCD_VER_START_AD, y1);
		writeRegister16(TFTLCD_VER_END_AD, y2);
		writeRegister16(TFTLCD_GRAM_HOR_AD, x); // Set address counter to top left
		writeRegister16(TFTLCD_GRAM_VER_AD, y);
	}
	else if (driver == ID_7575)
	{
		writeRegisterPair(HX8347G_COLADDRSTART_HI, HX8347G_COLADDRSTART_LO, x1);
		writeRegisterPair(HX8347G_ROWADDRSTART_HI, HX8347G_ROWADDRSTART_LO, y1);
		writeRegisterPair(HX8347G_COLADDREND_HI, HX8347G_COLADDREND_LO, x2);
		writeRegisterPair(HX8347G_ROWADDREND_HI, HX8347G_ROWADDREND_LO, y2);
	}
	else if ((driver == ID_9341) || (driver == ID_HX8357D))
	{
		uint32_t t = ((uint32_t) x1 << 16) | x2;
		writeRegister32(ILI9341_COLADDRSET, t);  // HX8357D uses same registers!
		t = ((uint32_t) y1 << 16) | y2;
		writeRegister32(ILI9341_PAGEADDRSET, t); // HX8357D uses same registers!
	}
}

void Adafruit_TFTLCD::drawFastHLine(int16_t x, int16_t y, int16_t length, uint16_t color)
{
	if ((int)y >= _height) return;
	if(driver == ID_C505)
	{
		drawFastLine(x, y, length, color, 0);
	}
	else
	{
		int16_t x2;

		// Initial off-screen clipping
		if ((length <= 0) || (y < 0) || (y >= _height) || (x >= _width)
				|| ((x2 = (x + length - 1)) < 0))
			return;

		if (x < 0)
		{        // Clip left
			length += x;
			x = 0;
		}
		if (x2 >= _width)
		{ // Clip right
			x2 = _width - 1;
			length = x2 - x + 1;
		}

		setAddrWindow(x, y, x2, y);
		flood(color, length);

		if (driver == ID_932X )
			setAddrWindow(0, 0, _width - 1, _height - 1);
		else if (driver == ID_7575)
			setLR();
	}
}

//------------------------------------------------------------------------------------------------
void Adafruit_TFTLCD::drawFastVLine(int16_t x, int16_t y, int16_t length, uint16_t color)
{
	if(driver == ID_C505)
	{
		if( (int)x >= _width) return;
		drawFastLine(x, y, length, color, 1);
	}
	else
	{
		int16_t y2;

		// Initial off-screen clipping
		if ((length <= 0) || (x < 0) || (x >= _width) || (y >= _height)
				|| ((y2 = (y + length - 1)) < 0))
			return;
		if (y < 0)
		{         // Clip top
			length += y;
			y = 0;
		}
		if (y2 >= _height)
		{ // Clip bottom
			y2 = _height - 1;
			length = y2 - y + 1;
		}

		setAddrWindow(x, y, x, y2);
		flood(color, length);

		if (driver == ID_932X )
			setAddrWindow(0, 0, _width - 1, _height - 1);
		else if (driver == ID_7575)
			setLR();
	}
}

//------------------------------------------------------------------------------------------------
void Adafruit_TFTLCD::fillRect(int16_t x1, int16_t y1, int16_t w, int16_t h, uint16_t fillcolor)
{
	if(driver == ID_C505)
	{
		while(h--)
		{
			if (y1 >= _height) return;
			drawFastLine(x1, y1++, w, fillcolor, 0);
		}
	}
	else
	{
		int16_t x2, y2;

		// Initial off-screen clipping
		if ((w <= 0) || (h <= 0) || (x1 >= _width) || (y1 >= _height) || ((x2 = x1 + w - 1) < 0)
				|| ((y2 = y1 + h - 1) < 0))
			return;
		if (x1 < 0)
		{ // Clip left
			w += x1;
			x1 = 0;
		}
		if (y1 < 0)
		{ // Clip top
			h += y1;
			y1 = 0;
		}
		if (x2 >= _width)
		{ // Clip right
			x2 = _width - 1;
			w = x2 - x1 + 1;
		}
		if (y2 >= _height)
		{ // Clip bottom
			y2 = _height - 1;
			h = y2 - y1 + 1;
		}

		setAddrWindow(x1, y1, x2, y2);
		flood(fillcolor, (uint32_t) w * (uint32_t) h);

		if (driver == ID_932X || driver == ID_C505)
			setAddrWindow(0, 0, _width - 1, _height - 1);
		else if (driver == ID_7575)
			setLR();
	}
}

//------------------------------------------------------------------------------------------------
void Adafruit_TFTLCD::fillScreen(uint16_t color)
{
	uint16_t x = 0, y = 0;
	if (driver == ID_932X || driver == ID_C505)
	{
		// For the 932X, a full-screen address window is already the default
		// state, just need to set the address pointer to the top-left corner.
		// Although we could fill in any direction, the code uses the current
		// screen rotation because some users find it disconcerting when a
		// fill does not occur top-to-bottom.
		switch (rotation)
		{
		case 1:
			x = TFTWIDTH - 1;
			y = 0;
			break;
		case 2:
			x = TFTWIDTH - 1;
			y = TFTHEIGHT - 1;
			break;
		case 3:
			x = 0;
			y = TFTHEIGHT - 1;
			break;
		case 0:
		default:
			x = 0;
			y = 0;
			break;
		}
		writeRegister16(TFTLCD_GRAM_HOR_AD, x);
		writeRegister16(TFTLCD_GRAM_VER_AD, y);
	}
	else if ((driver == ID_9341) || (driver == ID_7575) || (driver == ID_HX8357D))
	{
		// For these, there is no settable address pointer, instead the
		// address window must be set for each drawing operation.  However,
		// this display takes rotation into account for the parameters, no
		// need to do extra rotation math here.
		setAddrWindow(0, 0, _width - 1, _height - 1);

	}
	flood(color, (long) TFTWIDTH * (long) TFTHEIGHT);
}

//------------------------------------------------------------------------------------------------
void Adafruit_TFTLCD::drawPixel(int16_t x, int16_t y, uint16_t color)
{
	if ((x < 0) || (y < 0) || (x >= _width) || (y >= _height))
		return;

	if (driver == ID_932X || driver == ID_C505)
	{
		//Serial.print("drawPixel: rotation is ");
		//Serial.println(rotation, HEX);
		switch (rotation) {
		case 1:
		  swap(x, y);
		  x = TFTWIDTH - x - 1;
		  break;
		case 2:
		  x = TFTWIDTH - x - 1;
		  y = TFTHEIGHT - y - 1;
		  break;
		case 3:
		  swap(x, y);
		  y = TFTHEIGHT - y - 1;
		  break;
		}
		if ((x >= TFTWIDTH) || (y >= TFTHEIGHT)) return;
		writeRegister16(TFTLCD_GRAM_HOR_AD, x);
		writeRegister16(TFTLCD_GRAM_VER_AD, y);
		writeCommand((uint16_t)TFTLCD_RW_GRAM);
		writeData(color);
	}
	else if (driver == ID_7575)
	{
		uint8_t lo, table[4] = {0, 0x60,0xC0,0xA0};
		lo = table[rotation & 0x03];
		writeRegister8( HX8347G_MEMACCESS, lo);
		// Only upper-left is set -- bottom-right is full screen default
		writeRegisterPair(HX8347G_COLADDRSTART_HI, HX8347G_COLADDRSTART_LO, x);
		writeRegisterPair(HX8347G_ROWADDRSTART_HI, HX8347G_ROWADDRSTART_LO, y);
		writeCommand((uint8_t) TFTLCD_RW_GRAM);
		writeData(color);
	}
	else if ((driver == ID_9341) || (driver == ID_HX8357D))
	{
		setAddrWindow(x, y, _width - 1, _height - 1);
		writeCommand((uint16_t) 0x2C);
		writeData(color);
	}
}

//------------------------------------------------------------------------------------------------
void Adafruit_TFTLCD::setRotation(uint8_t x)
{
	// Call parent rotation func first -- sets up rotation flags, etc.
	Adafruit_GFX::setRotation(x);
	// Then perform hardware-specific rotation operations...

	if (driver == ID_932X || driver == ID_C505)
	{
			return;
		uint16_t t;
		switch (rotation)
		{
		default:
			t = 0x1030;
			break;
		case 1:
			t = 0x1028;
			break;
		case 2:
			t = 0x1020;
			break;
		case 3:
			t = driver == ID_932X ? 0x1018 : 0x1028;
			break;
		}
		writeRegister16(TFTLCD_ENTRY_MOD, t);
		//setAddrWindow(0, 0, _width - 1, _height - 1);
	}
	if (driver == ID_7575)
	{
		uint8_t lo, table[4] = {0, 0x60,0xC0,0xA0};
		lo = table[rotation & 0x03];
		writeRegister8(HX8347G_MEMACCESS, lo);
		// 7575 has to set the address window on most drawing operations.
		// drawPixel() cheats by setting only the top left...by default,
		// the lower right is always reset to the corner.
		setLR(); // CS_IDLE happens here
	}
	if (driver == ID_9341)
	{
		// MEME, HX8357D uses same registers as 9341 but different values
		uint16_t t = 0;

		switch (rotation)
		{
		case 2:
			t = ILI9341_MADCTL_MX | ILI9341_MADCTL_BGR;
			break;
		case 3:
			t = ILI9341_MADCTL_MV | ILI9341_MADCTL_BGR;
			break;
		case 0:
			t = ILI9341_MADCTL_MY | ILI9341_MADCTL_BGR;
			break;
		case 1:
			t = ILI9341_MADCTL_MX | ILI9341_MADCTL_MY | ILI9341_MADCTL_MV | ILI9341_MADCTL_BGR;
			break;
		}
		writeRegister8(ILI9341_MADCTL, t); // MADCTL
		// For 9341, init default full-screen address window:
		setAddrWindow(0, 0, _width - 1, _height - 1); // CS_IDLE happens here
	}

	if (driver == ID_HX8357D)
	{
		// MEME, HX8357D uses same registers as 9341 but different values
		uint16_t t = 0;
		switch (rotation)
		{
		case 2:
			t = HX8357B_MADCTL_RGB;
			break;
		case 3:
			t = HX8357B_MADCTL_MX | HX8357B_MADCTL_MV | HX8357B_MADCTL_RGB;
			break;
		case 0:
			t = HX8357B_MADCTL_MX | HX8357B_MADCTL_MY | HX8357B_MADCTL_RGB;
			break;
		case 1:
			t = HX8357B_MADCTL_MY | HX8357B_MADCTL_MV | HX8357B_MADCTL_RGB;
			break;
		}
		writeRegister8(ILI9341_MADCTL, t); // MADCTL
		// For 8357, init default full-screen address window:
		setAddrWindow(0, 0, _width - 1, _height - 1); // CS_IDLE happens here
	}
}
//------------------------------------------------------------------------------------------------
// Because this function is used infrequently, it configures the ports for
// the read operation, reads the data, then restores the ports to the write
// configuration.  Write operations happen a LOT, so it's advantageous to
// leave the ports in that state as a default.
uint16_t Adafruit_TFTLCD::readPixel(int16_t x, int16_t y)
{
	uint16_t data;
	if ((x < 0) || (y < 0) || (x >= _width) || (y >= _height))
		return 0;

	if (driver == ID_932X || driver == ID_C505)
	{
		switch (rotation) {
		case 1:
		  swap(x, y);
		  x = TFTWIDTH - x - 1;
		  break;
		case 2:
		  x = TFTWIDTH - x - 1;
		  y = TFTHEIGHT - y - 1;
		  break;
		case 3:
		  swap(x, y);
		  y = TFTHEIGHT - y - 1;
		  break;
		}
		writeRegister16(TFTLCD_GRAM_HOR_AD, x);
		writeRegister16(TFTLCD_GRAM_VER_AD, y);
		if(driver == ID_932X )
		{
			// Inexplicable thing: sometimes pixel read has high/low bytes
			// reversed.  A second read fixes this.  Unsure of reason.  Have
			// tried adjusting timing in read8() etc. to no avail.
			for (uint8_t pass = 0; pass < 2; pass++)
			{
				writeCommand((uint8_t) 0x00);
				writeCommand((uint8_t) TFTLCD_RW_GRAM);
				data = readData();
				data = readData();
			}
		}
		else
		{
			writeCommand((uint8_t) 0x00);
			writeCommand((uint8_t) TFTLCD_RW_GRAM);
			data = readData();
		}
		return data;
	}
	else if (driver == ID_7575)
	{
		//uint8_t r, g, b;
		writeRegisterPair(HX8347G_COLADDRSTART_HI, HX8347G_COLADDRSTART_LO, x);
		writeRegisterPair(HX8347G_ROWADDRSTART_HI, HX8347G_ROWADDRSTART_LO, y);
		writeCommand((uint8_t) TFTLCD_RW_GRAM);
		data = readData();
		data = readData();
		//return (((uint16_t) r & B11111000) << 8) | (((uint16_t) g & B11111100) << 3) | (b >> 3);
		return data;
	}
	return 0;
}

//------------------------------------------------------------------------------------------------
// Unlike the 932X drivers that set the address window to the full screen
// by default (using the address counter for drawPixel operations), the
// 7575 needs the address window set on all graphics operations.  In order
// to save a few register writes on each pixel drawn, the lower-right
// corner of the address window is reset after most fill operations, so
// that drawPixel only needs to change the upper left each time.
void Adafruit_TFTLCD::setLR(void)
{
	writeRegisterPair(HX8347G_COLADDREND_HI, HX8347G_COLADDREND_LO, _width - 1);
	writeRegisterPair(HX8347G_ROWADDREND_HI, HX8347G_ROWADDREND_LO, _height - 1);
}

//------------------------------------------------------------------------------------------------
uint16_t Adafruit_TFTLCD::readID(void)
{
	uint16_t id = readRegister32(0xD3);
	if (id == 0x9341)
		return id;
	if(id == 0x0101)
		return 0x9341;
	id = readRegister(0);
	if (id == 0xC505)
		return id;
	if (readRegister32(0x04) == 0x8000)
	{ // close enough
		writeRegister24(HX8357D_SETC, 0xFF8357);
		delay(300);
		if (readRegister32(0xD0) == 0x990000)
		{
			return 0x8357;
		}
	}
	return readRegister(0);
}

//------------------------------------------------------------------------------------------------
// Fast block fill operation for fillScreen, fillRect, H/V line, etc.
// Requires setAddrWindow() has previously been called to set the fill
// bounds.  'len' is inclusive, MUST be >= 1.
void Adafruit_TFTLCD::flood(uint16_t color, uint32_t len)
{
	if (driver == ID_9341)
	{
		writeCommand((uint16_t) 0x2C); //write8(0x2C);
	}
	else if (driver == ID_932X || driver == ID_C505)
	{
		writeCommand((uint16_t) TFTLCD_RW_GRAM);
	}
	else if (driver == ID_HX8357D)
	{
		writeCommand((uint16_t) HX8357_RAMWR);
	}

	while (len--)
	{
		writeData(color);
	}

	return;
}

//------------------------------------------------------------------------------------------------
// Pass 8-bit (each) R,G,B, get back 16-bit packed color
uint16_t Adafruit_TFTLCD::color565(uint8_t r, uint8_t g, uint8_t b)
{
	return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

//------------------------------------------------------------------------------------------------
void Adafruit_TFTLCD::reset()
{
	void resetDirection();
	if (_reset)
	{
		digitalWrite(_reset, LOW);
		delay(2);
		digitalWrite(_reset, HIGH);
	}
	writeCommand((uint8_t) 0x00);
	for (uint8_t i = 0; i < 3; i++)
		writeData((uint8_t) 0); // Three extra 0x00s
}
//------------------------------------------------------------------------------------------------
