// Touch screen library with X Y and Z (pressure) readings as well
// as oversampling to avoid 'bouncing'
// (c) ladyada / adafruit
// Code under MIT License
#ifndef _TOUCHSCREEN_H_
#define _TOUCHSCREEN_H_

#define MINPRESSURE 10
#define MAXPRESSURE 1000

#define SCREEN_WIDTH   240
#define SCREEN_HEIGHT  320

// Pin set used for display id 0xC505
#define XP_PIN_ALT  6
#define YP_PIN_ALT A1
#define XM_PIN_ALT A2
#define YM_PIN_ALT  7

// Pin set used by other display
#define XP_PIN 8
#define YP_PIN A3
#define XM_PIN A2
#define YM_PIN 9

#define RANGE_X1  (730)
#define OFFSET_X1 (130)

#define RANGE_Y1  (520)
#define OFFSET_Y1 (320)

#define RANGE_X  (680)
#define OFFSET_X (110)

#define RANGE_Y	 (780)
#define OFFSET_Y (100)

//#define TS_MINX 110
//#define TS_MAXX 780
//#define TS_MINY 120
//#define TS_MAXY 900


//#define TS_MIN_X 120
//#define TS_MAX_X 940
//
//#define TS_MIN_Y 110
//#define TS_MAX_Y 780

class Point {
public:
	Point(void);
	Point(int16_t x, int16_t y, int16_t z);
	bool operator==(Point);
	bool operator!=(Point);
	int16_t x, y, z;
};

class TouchScreen {
public:
	TouchScreen(uint16_t rxPlate = 0);
	void begin(uint16_t id);
	bool isTouching(void);
	uint16_t pressure(void);
	int readTouchY();
	int readTouchX();
	Point getPoint();
	void ScaleIt(Point &point);
	void processTheKey(char c);

	uint8_t m_XP_PIN, m_YP_PIN, m_XM_PIN, m_YM_PIN;

private:
	uint8_t m_XP_PORT, m_YP_PORT, m_XM_PORT, m_YM_PORT;
	uint8_t m_XP_MASK, m_YP_MASK, m_XM_MASK, m_YM_MASK;
	int		m_RangeX, m_OffSetX, m_RangeY, m_OffSetY;

	uint16_t m_rxPlate;
	uint16_t m_DriverId;
	int16_t m_PressureThreshhold;
};

#endif // _TOUCHSCREEN_H_
