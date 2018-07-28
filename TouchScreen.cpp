// Touch screen library with X Y and Z (pressure) readings as well
// as oversampling to avoid 'bouncing'
// (c) ladyada / adafruit
// Code under MIT License

#include "pins_arduino.h"
#include "wiring_private.h"
#include <avr/pgmspace.h>
#include "TouchScreen.h"

//int TS_MIN_X=330;
//int TS_MAX_X=990;
//int TS_RANGE_X= 730;
//int TS_OFFSET_X=130;
//
//int TS_RANGE_Y= 630;
//int TS_OFFSET_Y=270;

//int TS_MIN_Y=310;
//int TS_MAX_Y=870;

void TouchScreen::processTheKey(char c)
{
	int i = 10;
	switch(c)
	{
	case	'a' : m_RangeX -=i; break;
	case	'A' : m_RangeX +=i; break;

	case	'b' : m_OffSetX -=i; break;
	case	'B' : m_OffSetX +=i; break;

	case	'c' : m_RangeY -=i; break;
	case	'C' : m_RangeY +=i; break;

	case	'd' : m_OffSetY -=i; break;
	case	'D' : m_OffSetY +=i; break;
	};
	char temp[80];
	sprintf(temp, "RangeX: %d OffsetX: %d RangeY: %d  OffsetY: %d ", m_RangeX, m_OffSetX, m_RangeY, m_OffSetY);
	Serial.println(temp);
}
// increase or decrease the touch screen over sampling. This is a little different than you make think:
// 1 is no over sampling, whatever data we get is immediately returned
// 2 is double-sampling and we only return valid data if both points are the same
// 3+ uses insert sort to get the median value.
// We found 2 is precise yet not too slow so we suggest sticking with it!

#define NUMSAMPLES 3

Point::Point(void) :
		x(0), y(0), z(0)
{
}
Point::Point(int16_t X, int16_t Y, int16_t Z) :
		x(X), y(Y), z(Z)
{
}
//---------------------------------------------------------------------------------------
bool Point::operator==(Point p1)
{
	return ((p1.x == x) && (p1.y == y) && (p1.z == z));
}
//---------------------------------------------------------------------------------------
bool Point::operator!=(Point p1)
{
	return ((p1.x != x) || (p1.y != y) || (p1.z != z));
}
//---------------------------------------------------------------------------------------
#if (NUMSAMPLES > 2)
static void insert_sort(int array[], uint8_t size)
{
	uint8_t j;
	int save;

	for (int i = 1; i < size; i++)
	{
		save = array[i];
		for (j = i; j >= 1 && save < array[j - 1]; j--)
			array[j] = array[j - 1];
		array[j] = save;
	}
}
#endif

//---------------------------------------------------------------------------------------
TouchScreen::TouchScreen(uint16_t rxplate) :
			m_XP_PIN(0),m_YP_PIN(0),m_XM_PIN(0),m_YM_PIN(0),
			m_XP_PORT(0),m_YP_PORT(0),m_XM_PORT(0),m_YM_PORT(0),
			m_XP_MASK(0),m_YP_MASK(0),m_XM_MASK(0),m_YM_MASK(0),
			m_rxPlate(rxplate)
{
	m_PressureThreshhold = MINPRESSURE;
	m_DriverId = 0;
	m_OffSetY = m_RangeY = m_OffSetX = m_RangeX= 0;
}

//---------------------------------------------------------------------------------------
void TouchScreen::begin(uint16_t id)
{
	m_DriverId = id;
	bool set1 = (id == 0xC505);
	m_XP_PIN = set1 ? XP_PIN_ALT : XP_PIN;
	m_YP_PIN = set1 ? YP_PIN_ALT : YP_PIN;
	m_XM_PIN = set1 ? XM_PIN_ALT : XM_PIN;
	m_YM_PIN = set1 ? YM_PIN_ALT : YM_PIN;

	m_XP_PORT = digitalPinToPort(m_XP_PIN);
	m_YP_PORT = digitalPinToPort(m_YP_PIN);
	m_XM_PORT = digitalPinToPort(m_XM_PIN);
	m_YM_PORT = digitalPinToPort(m_YM_PIN);

	m_XP_MASK = digitalPinToBitMask(m_XP_PIN);
	m_YP_MASK = digitalPinToBitMask(m_YP_PIN);
	m_XM_MASK = digitalPinToBitMask(m_XM_PIN);
	m_YM_MASK = digitalPinToBitMask(m_YM_PIN);

	m_RangeX  = set1 ? RANGE_X1  : RANGE_X;
	m_OffSetX = set1 ? OFFSET_X1 : OFFSET_X;
	m_RangeY  = set1 ? RANGE_Y1  : RANGE_Y;
	m_OffSetY = set1 ? OFFSET_Y1 : OFFSET_Y;
}

void TouchScreen::ScaleIt(Point &point)
{
//		char temp[40];
//		sprintf(temp, " %d * %d *  Z:%d ### ", point.x , point.y, point.z );
//		Serial.print(temp);

		float m;
		m = (float)(point.x - m_OffSetX)/(float)(m_RangeX);
		point.x = SCREEN_WIDTH * m ;
		m = (float)(point.y - m_OffSetY)/(float)(m_RangeY);
		point.y = SCREEN_HEIGHT * m;

//		sprintf(temp, "Test %d * %d \n", point.x , point.y );
//		Serial.println(temp);

}
//---------------------------------------------------------------------------------------
Point TouchScreen::getPoint(void)
{
	int x, y, z;
	int samples[NUMSAMPLES];
	uint8_t i, valid = 1;

	pinMode(m_YP_PIN, INPUT);
	pinMode(m_YM_PIN, INPUT);
	*portOutputRegister(m_YP_PORT) &= ~m_YP_MASK;
	*portOutputRegister(m_YM_PORT) &= ~m_YM_MASK;

	pinMode(m_XP_PIN, OUTPUT);
	pinMode(m_XM_PIN, OUTPUT);
	*portOutputRegister(m_XP_PORT) |= m_XP_MASK;
	*portOutputRegister(m_XM_PORT) &= ~m_XM_MASK;

	for (i = 0; i < NUMSAMPLES; i++)
	{
		samples[i] = analogRead(m_YP_PIN);
	}
#if NUMSAMPLES > 2
	insert_sort(samples, NUMSAMPLES);
#endif
#if NUMSAMPLES == 2
	if (samples[0] != samples[1])
		valid = 0;
#endif
	//x = (1023 - samples[NUMSAMPLES/2]);
	x = samples[NUMSAMPLES/ 2];
	pinMode(m_XP_PIN, INPUT);
	pinMode(m_XM_PIN, INPUT);
	*portOutputRegister(m_XP_PORT) &= ~m_XP_MASK;

	pinMode(m_YP_PIN, OUTPUT);
	*portOutputRegister(m_YP_PORT) |= m_YP_MASK;

	pinMode(m_YM_PIN, OUTPUT);

	for (i = 0; i < NUMSAMPLES; i++)
	{
		samples[i] = analogRead(m_XM_PIN);
	}

#if NUMSAMPLES > 2
	insert_sort(samples, NUMSAMPLES);
#endif
#if NUMSAMPLES == 2
	if (samples[0] != samples[1])
		valid = 0;
#endif
	//y = (1023 - samples[NUMSAMPLES/2]);
	y = samples[NUMSAMPLES/2];

	pinMode(m_XP_PIN, OUTPUT);
	*portOutputRegister(m_XP_PORT) &= ~m_XP_MASK;	// Set X+ to ground
	*portOutputRegister(m_YM_PORT) |= m_YM_MASK;	// Set Y- to VCC
	*portOutputRegister(m_YP_PORT) &= ~m_YP_MASK;	// Hi-Z X- and Y+
	pinMode(m_YP_PIN, INPUT);

	int z1 = analogRead(m_XM_PIN);
	int z2 = analogRead(m_YP_PIN);

	//Serial.print("z1:"); Serial.print(z1,DEC);
	//Serial.print(" z2:"); Serial.println(z2,DEC);
	if (m_rxPlate != 0)
	{
		float rtouch;
		rtouch = z2;
		if(z1 > 5)
		{
			rtouch /= (float)z1;
			rtouch -= 1;
			rtouch *= (float)(1024-x);
			rtouch *= (float) m_rxPlate;
			rtouch /= (float)1024;
			z = rtouch;
		}
		else
			z = 0;
	}
	else
	{
		z = abs(z2 - z1);
	}

	if (!valid)
	{
		z = 0;
	}

	return Point(x, y, z);
}

//---------------------------------------------------------------------------------------
int TouchScreen::readTouchX(void)
{
	pinMode(m_YP_PIN, INPUT);
	pinMode(m_YM_PIN, INPUT);
	digitalWrite(m_YP_PIN, LOW);
	digitalWrite(m_YM_PIN, LOW);

	pinMode(m_XP_PIN, OUTPUT);
	digitalWrite(m_XP_PIN, HIGH);
	pinMode(m_XM_PIN, OUTPUT);
	digitalWrite(m_XM_PIN, LOW);

	return (analogRead(m_YP_PIN)); //0-1023
}

//---------------------------------------------------------------------------------------
int TouchScreen::readTouchY(void)
{
	pinMode(m_XP_PIN, INPUT);
	pinMode(m_XM_PIN, INPUT);
	digitalWrite(m_XP_PIN, LOW);
	digitalWrite(m_XM_PIN, LOW);

	pinMode(m_YP_PIN, OUTPUT);
	digitalWrite(m_YP_PIN, HIGH);
	pinMode(m_YM_PIN, OUTPUT);
	digitalWrite(m_YM_PIN, LOW);

	return (analogRead(m_XM_PIN)); //0-1023
}

//---------------------------------------------------------------------------------------
uint16_t TouchScreen::pressure(void)
{
	// Set X+ to ground
	pinMode(m_XP_PIN, OUTPUT);
	digitalWrite(m_XP_PIN, LOW);

	// Set Y- to VCC
	pinMode(m_YM_PIN, OUTPUT);
	digitalWrite(m_YM_PIN, HIGH);

	// Hi-Z X- and Y+
	digitalWrite(m_XM_PIN, LOW);
	pinMode(m_XM_PIN, INPUT);
	digitalWrite(m_YP_PIN, LOW);
	pinMode(m_YP_PIN, INPUT);

	int z2 = analogRead(m_XM_PIN);
	int z1 = analogRead(m_YP_PIN);

	if (m_rxPlate != 0)
	{
		// now read the x
		float rtouch;
		rtouch = z2;
		if(z1 != 0)
			rtouch /= z1;
		rtouch -= 1.0;
		rtouch *= readTouchX();
		rtouch *= m_rxPlate;
		rtouch /= 1024.0;

		return rtouch;
	}
	else
	{
		return (1023 - (z2 - z1));
	}
}
//---------------------------------------------------------------------------------------
