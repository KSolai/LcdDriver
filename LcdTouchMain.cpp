#include "Adafruit_GFX.h"    // Core graphics library
#include "Adafruit_TFTLCD.h" // Hardware-specific library
#include "TouchScreen.h"
#include "SPI.h"
#include "SD.h"
#include "Ping.h"
#include "wire.h"

#ifdef MEGA //__AVR_ATmega2560__
#include <Servo.h>
#endif

#define REPORTBENCHMARK 0
#if (REPORTBENCHMARK)
void reportBenchMark();
#endif

#define CHECKSDCARD 0
#if (CHECKSDCARD)
void CheckSDCard();
#endif

void showImage(char * fileName);
//void testMe(short request);

// When using the BREAKOUT BOARD only, use these 8 data lines to the LCD:
// For the Arduino Uno, Duemilanove, Diecimila, etc.:
//   D0 connects to digital pin 8  (Notice these are
//   D1 connects to digital pin 9   NOT in order!)
//   D2 connects to digital pin 2
//   D3 connects to digital pin 3
//   D4 connects to digital pin 4
//   D5 connects to digital pin 5
//   D6 connects to digital pin 6
//   D7 connects to digital pin 7
// For the Arduino Mega, use digital pins 22 through 29
// (on the 2-row header at the end of the board).

// The control pins for the LCD can be assigned to any digital or
// analog pins...but we'll use the analog pins as this allows us to
// double up the pins with the touch screen (see the TFT paint example).
#define LCD_CS A3 // Chip Select goes to Analog 3
#define LCD_CD A2 // Command/Data goes to Analog 2
#define LCD_WR A1 // LCD Write goes to Analog 1
#define LCD_RD A0 // LCD Read goes to Analog 0

#define LCD_RESET A4 // Can alternately just connect to Arduino's reset pin

// Assign human-readable names to some common 16-bit color values:
#define	BLACK   0x0000
#define	BLUE    0x001F
#define	RED     0xF800
#define	GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF


#define BOXSIZE 30
enum Mode {
	NORMAL, HAIRLINE_CROSS, SOLID, INVALID
};

enum KeyState {
	NOWAIT, MINWAIT, MAXWAIT
};

#define SdChipSelect (10)

bool SDOK = false;
uint32_t bmpImageOffset = 0;
short ImageCnt = 0;
#define BUFFPIXEL       60		// must be a divisor of 240
#define BUFFPIXEL_X3    180		// BUFFPIXELx3

// For better pressure precision, we need to know the resistance
// between X+ and X- Use any Multimeter to read it
// For the one we're using, its 300 Ohm's across the X plate
TouchScreen ts = TouchScreen(300);
Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);

//Global objects and variables
uint16_t screenColor, currentColor, lastColor;
uint16_t myColors[8] = { BLACK, BLUE, RED, GREEN, CYAN, MAGENTA, YELLOW, WHITE };
short colorIndex = 0;

int minx=300,maxx=300,miny=600,maxy=300;
uint16_t count = 0;
Mode mode = NORMAL;
uint16_t penColor;
long timer1_counter;
unsigned long ticks = 0;
Ping cPing;

#ifdef MEGA
unsigned long serviceTicks = 20;
unsigned long serviceTicks1 = 10;
unsigned long lastPingVal =0;
unsigned long maxPingVal =50;
unsigned long servoPos = 0;

#define USSPin 22
#define SERVOPin 24
#define POTPin A8
#define MAX_VAL 1023
#define INC_VAL 5
int Count50us(0);
#define TRIGPIN A15
#define ECHOPIN A14

void ServiceDistanceSener();
void myPingServiceTask();
void myServoServiceTask();
void showDistance();
unsigned long ping();
Servo servo;
#endif

KeyState keyState = NOWAIT;
short lastKey;
short keyMinWait = 0;

//-----------------------------------------------------------------------------
ISR(TIMER1_OVF_vect)        // interrupt service routine
{
	TCNT1 = timer1_counter;   // pre-load timer
	ticks++;
	if (keyMinWait != 0)
		keyMinWait--;
	cPing.Service();
#ifdef MEGA
	if(serviceTicks != 0)
		if(--serviceTicks == 0)
		{
			//Count50us++;
			//(Count50us & 1) ? digitalWrite(USSPin, LOW): digitalWrite(USSPin, HIGH);
			//serviceTicks = 50;
			//myPingServiceTask();
			//ServiceDistanceSener();
		}
	if(serviceTicks1 != 0)
		if(--serviceTicks1 == 0)
		{
			//myServoServiceTask();
		}
#endif
}
//-----------------------------------------------------------------------------
void myPrintStr(char * tmp){ Serial.println(tmp); }
void myPrintNum(int tmp){ Serial.println(tmp); }
//-----------------------------------------------------------------------------
unsigned short nextColor()
{
	return myColors[(++colorIndex &= 0x07)];
}
//-----------------------------------------------------------------------------
bool DebounceCheck(short key)
{
	bool keyStatus = false;
	switch (keyState)
	{
	case NOWAIT:
		if (key != lastKey)
		{
			lastKey = key;
			keyState = MINWAIT;
			keyMinWait = 10;
		}
		break;
	case MINWAIT:
		if (key != lastKey)
			keyState = NOWAIT;
		else if ((keyStatus = keyMinWait) == 0)
		{
			keyState = MAXWAIT;
			keyMinWait = 50;
		}
		break;
	case MAXWAIT:
		if (keyMinWait == 0)
		{
			keyState = NOWAIT;
			lastKey = 10;
		}
		break;
	}
	return keyStatus;
}

//---------------------------------------------------------------------------------------
unsigned short GetPenColor()
{
	switch (screenColor)
	{
	case BLACK: 	return WHITE;
	case BLUE:		return YELLOW;
	case GREEN:		return RED;
	case CYAN:		return BLUE;
	case MAGENTA:	return BLUE;
	case YELLOW:	return GREEN;
	case RED:
	case WHITE:
	default:		return BLACK;
	}
}
//-----------------------------------------------------------------------------
void setup(void) {
	Serial.begin(115200);
	pinMode(USSPin, OUTPUT);
	//-------------------------------------------------------------------------
	// initialize timer1
	noInterrupts();
	// disable all interrupts
	TCCR1A = 0;
	TCCR1B = 0;
	// Set timer1_counter to the correct value for our interrupt interval
	timer1_counter =-3;
	//timer1_counter = 64911;   // preload timer 65536-16MHz/256/100Hz
	//timer1_counter = 64286;   // preload timer 65536-16MHz/256/50Hz
	//timer1_counter = 34286;   // preload timer 65536-16MHz/256/2Hz
	TCNT1 = timer1_counter;   // preload timer
	TCCR1B |= (1 << CS12);    // 256 prescaler
	TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
	interrupts();
	// enable all interrupts
	//-------------------------------------------------------------------------

	Serial.print("TFT LCD test Using Adafruit 2.4 ");

#ifdef USE_ADAFRUIT_SHIELD_PINOUT
	Serial.println("TFT Arduino Shield");
#else
	Serial.println("TFT Breakout Board");
#endif

	Serial.print("TFT size is ");
	Serial.print(tft.width());
	Serial.print("x");
	Serial.println(tft.height());

	tft.reset();

	uint16_t identifier = tft.readID();

//	char * found = (char*) "Found ";
//	char * lcdDriver = (char*) " LCD Driver Chip";
//
//	if (identifier == 0x9325){
//		Serial.print(found); Serial.print("ILI9325"); Serial.println(lcdDriver);
//	} else if (identifier == 0x4535){
//		Serial.print(found); Serial.print("LGDP4535"); Serial.println(lcdDriver);
//	} else if (identifier == 0x9328){
//		Serial.print(found); Serial.print("ILI9328"); Serial.println(lcdDriver);
//	} else if (identifier == 0x7575){
//		Serial.print(found); Serial.print("HX8347G"); Serial.println(lcdDriver);
//	} else if (identifier == 0x9341){
//		Serial.print(found); Serial.print("ILI9341"); Serial.println(lcdDriver);
//	} else if (identifier == 0x8357){
//		Serial.print(found); Serial.print("HX8357D"); Serial.println(lcdDriver);
//	} else if (identifier == 0xC505){
//		Serial.print(found); Serial.print("C505"); Serial.println(lcdDriver);
//	} else{
//		Serial.print(F("Unknown LCD driver chip: "));
//		Serial.println(identifier, HEX);
//		Serial.println(F("Make sure break-out or shield boards defines set correctly"));
//		Serial.println(F(" in \"Adafruit_TFT.h\" or double-check all wiring"));
//		return;
//	}

	tft.begin(identifier);
	ts.begin(identifier);

	currentColor = RED;
	pinMode(13, OUTPUT);

	tft.fillScreen(currentColor);
	tft.setRotation(0);

#if REPORTBENCHMARK
	reportBenchMark();
#endif

	pinMode(SdChipSelect, OUTPUT);
	Serial.print("Initializing SD card...");

#if (CHECKSDCARD)
	CheckSDCard();
#endif

	if(!SD.begin(SdChipSelect))
	{
		Serial.println("initialization failed!");
	}
	else
	{
		Serial.println("initialization done.");
		SDOK = true;
	}

#ifdef MEGA
	servo.attach(SERVOPin);
	pinMode(POTPin, INPUT);
	//pinMode(TRIGPIN, OUTPUT);
	//pinMode(ECHOPIN, INPUT);
	cPing.Setup(TRIGPIN, ECHOPIN, HIGH);
	//myServiceTask();
#endif
}
//-----------------------------------------------------------------------------
#if (CHECKSDCARD)
void CheckSDCard()
{
	Sd2Card card;
	SdVolume volume;
	SdFile root;
	if(!card.init(SPI_HALF_SPEED, SdChipSelect)){
		Serial.println("initialization failed. Things to check:");
		Serial.println("* is a card is inserted?");
		Serial.println("* Is your wiring correct?");
		Serial.println("* Is chipSelect pin used as per the module?");
		return;
	}else{
		Serial.println("Wiring is correct and a card is present.");
	}

	Serial.print("\nCard type: ");
	switch(card.type()) {
		case SD_CARD_TYPE_SD1:
		  Serial.println("SD1");
		  break;
		case SD_CARD_TYPE_SD2:
		  Serial.println("SD2");
		  break;
		case SD_CARD_TYPE_SDHC:
		  Serial.println("SDHC");
		  break;
		default:
		  Serial.println("Unknown");
	}

	if(!volume.init(card))
	{
		Serial.println("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card");
		return;
	}
	// print the type and size of the first FAT-type volume
	uint32_t volumesize;
	Serial.print("\nVolume type is FAT");
	Serial.println(volume.fatType(), DEC);
	Serial.println();

	volumesize = volume.blocksPerCluster();   // clusters are collections of blocks
	volumesize *= volume.clusterCount();      // we'll have a lot of clusters
	volumesize *= 512;                        // SD card blocks are always 512 bytes
	Serial.print("Volume size (bytes): ");
	Serial.println(volumesize);
	Serial.print("Volume size (Kbytes): ");
	volumesize /= 1024;
	Serial.println(volumesize);
	Serial.print("Volume size (Mbytes): ");
	volumesize /= 1024;
	Serial.println(volumesize);


	Serial.println("\nFiles found on the card (name, date and size in bytes): ");
	root.openRoot(volume);

	// list all files in the card with date and size
	root.ls(LS_R | LS_DATE | LS_SIZE);

	//showImage((char*)"MINIWOOF.BMP");
	//showImage((char*)"Johnny.bmp");
	delay(5000);
}
#endif

//-----------------------------------------------------------------------------
void loop(void) {

	digitalWrite(13, HIGH);
	Point p = ts.getPoint();
	digitalWrite(13, LOW);

	pinMode(ts.m_XM_PIN, OUTPUT);
	pinMode(ts.m_YP_PIN, OUTPUT);
	pinMode(ts.m_YM_PIN, OUTPUT);

	if ((count++ %0x2000) == 0)
	{
		count = 0xD000;
		lastColor = screenColor;
		screenColor = currentColor;
		if(ImageCnt++ & 1)
			showImage((char*)"JOHN03.BMP");
		else if(ImageCnt & 2)
			showImage((char*)"Johnny.bmp");
		else
			tft.fillScreen(currentColor);
		currentColor = nextColor();
	}

//	char input = Serial.read();
//	if(input != -1)
//	{
//		ts.processTheKey(input);
//	}

	// we have some minimum pressure we consider 'valid'
	// pressure of 0 means no pressing!
	if(p.z > MINPRESSURE && p.z < MAXPRESSURE)
	{
//		{
//			char temp[40];
//			if(p.x < minx) minx = p.x;
//			if(p.x > maxx) maxx = p.x;
//			if(p.y < miny) miny = p.y;
//			if(p.y > maxy) maxy = p.y;
//
//			Serial.print("Before ");
//			sprintf(temp, "%d - %d - %d  %d \n", minx, maxx, miny, maxy);
//			Serial.println(temp);
//		}

		ts.ScaleIt(p);

		if (abs(p.y) < BOXSIZE)
		{
			short m = p.x / BOXSIZE;
			if (DebounceCheck(m))
			{
				//Serial.println(m, DEC);
				tft.drawRect(BOXSIZE * m, 0, BOXSIZE, BOXSIZE, WHITE);
				tft.fillRect(BOXSIZE * m, 0, BOXSIZE, BOXSIZE, myColors[m]);
				if (m == 0)
				{
					mode = mode == NORMAL ? HAIRLINE_CROSS :
							mode == HAIRLINE_CROSS ? SOLID : NORMAL;
				}
				//testMe(m);
			}
		}

		if (abs(p.y) > BOXSIZE)
		{
			penColor = GetPenColor();
			switch (mode)
			{
			case NORMAL:
				for (int x = p.x; x < p.x + 1; x++)
					for (int y = p.y; y < p.y + 1; y++)
						tft.drawPixel(x, y, penColor);
				break;
			case HAIRLINE_CROSS:
				tft.drawFastHLine(0, p.y, tft.width(), penColor);
				tft.drawFastVLine(p.x, 0, tft.height(), penColor);
				delayMicroseconds(3000);
				tft.drawFastHLine(0, p.y, tft.width(), screenColor);
				tft.drawFastVLine(p.x, 0, tft.height(), screenColor);
				break;
			case SOLID:
				tft.fillCircle(p.x, p.y, 1, penColor);
				break;
			default:
				;
			}
		}
	}
	//if(serviceTicks == 0)
		//ServiceDistanceSener();
	if(serviceTicks == 0)
		showDistance();
	if(serviceTicks1 == 0)
	{
		lastPingVal = cPing.GetDuration();
		myServoServiceTask();
	}

}

//---------------------------------------------------------------------------------------
void showDistance()
{
	//double	distanceCms = cPing.GetDistanceInCms();
	//double	distanceInches = cPing.GetDistanceInInches();
	long duration = cPing.GetDuration();
	double	distanceCms = (duration*0.034)/2.0;
	double	distanceInches = (duration*0.0133)/2.0;
	Serial.print("Duartion: ");
	Serial.print(duration);
	Serial.print(" Distance: ");
	Serial.print(distanceInches);
	Serial.print(" inches Distance: ");
	Serial.print(distanceCms);
	Serial.print(" cms");
	Serial.println();
	noInterrupts();
	serviceTicks = 10000;
	interrupts();
}

//---------------------------------------------------------------------------------------
unsigned long testFillScreen() {
	unsigned long time = 0;
	for(short x=7; x >0; --x)
	{
		unsigned long start = micros();
		tft.fillScreen(myColors[x]);
		time += (micros() - start);
		delay(1000);
	}
	return time;
}

//---------------------------------------------------------------------------------------
void bmpdraw(File &f, int x, int y)
{
    f.seek(bmpImageOffset);
    //uint32_t time = millis();

    uint8_t sdbuffer[BUFFPIXEL_X3]; // 3 * pixels to buffer

    for (int i=0; i< TFTHEIGHT; i++) {
        for(int j=0; j<(240/BUFFPIXEL); j++)
        {
            f.read(sdbuffer, BUFFPIXEL_X3);

            uint8_t buffidx = 0;
            int offset_x = j*BUFFPIXEL;
            unsigned int __color[BUFFPIXEL];

            for(int k=0; k<BUFFPIXEL; k++) {
                __color[k] = sdbuffer[buffidx+2]>>3;                        // read
                __color[k] = __color[k]<<6 | (sdbuffer[buffidx+1]>>2);      // green
                __color[k] = __color[k]<<5 | (sdbuffer[buffidx+0]>>3);      // blue

                buffidx += 3;
                tft.drawPixel(k+offset_x, i,__color[k]);
            }
        }
    }

    //Serial.print(millis() - time, DEC);
    //Serial.println(" ms");
}

boolean bmpReadHeader(File &f)
{
    uint32_t tmp;
    uint16_t bmpDepth, tmp1;
    f.read(&tmp1,2);
    //Serial.print("magic "); Serial.println(tmp1, HEX);
    if(tmp1 != 0x4D42) // magic bytes missing
        return false;
    f.read(&tmp,4);	// read file size   //tmp = read32(f);
    //Serial.print("size:"); Serial.println(tmp, DEC);
    f.read(&tmp,4);	// read and ignore creator bytes

    f.read(&bmpImageOffset,4);   //bmpImageOffset = read32(f);
    //Serial.print("offset "); Serial.println(bmpImageOffset, DEC);

    f.read(&tmp,4); //tmp = read32(f); // read DIB header
    //Serial.print("header size "); Serial.println(tmp, DEC);

    uint32_t bmp_width;  //= read32(f);
    uint32_t bmp_height; //= read32(f);
    f.read(&bmp_width,4);
    f.read(&bmp_height,4);
    //Serial.print("bmp_width "); Serial.println(bmp_width, DEC);
    //Serial.print("bmp_height "); Serial.println(bmp_height, DEC);

    // if image is not 320x240, return false
    if(bmp_width != TFTWIDTH || bmp_height != TFTHEIGHT)
        return false;

    f.read(&tmp1,2);
    if(tmp1 != 1)
    	return false;

    f.read(&bmpDepth,2); //bmpDepth = read16(f);
    //Serial.print("bmpDepth "); Serial.println(bmpDepth, DEC);
    f.read(&tmp,4);
    if(tmp != 0) 	// compression not supported!
        return false;

    //Serial.print("compression ");  Serial.println(tmp, DEC);
    return true;
}

void showImage(char * fileName)
{
	if(!SDOK)
		return;

	String aFileName(fileName);

	File aFile = SD.open(aFileName);

    if(aFile.size() == 0)
    {
        Serial.println("didn't find file");
        //tft.setTextColor(WHITE); tft.setTextSize(1);
        //tft.println("didnt find BMPimage");
        return;
    }

    if(!bmpReadHeader(aFile)){
        Serial.println("bad bmp");
        //tft.setTextColor(WHITE); tft.setTextSize(1);
        //tft.println("bad bmp");
        aFile.close();
        return;
    }
    //int sz = aFile.size();

    bmpdraw(aFile, 0, 0);
    //Serial.println("Done with Draw");
    aFile.close();
    delay(2000);
}

#if REPORTBENCHMARK
unsigned long testText();
unsigned long testLines(uint16_t color);
unsigned long testFastLines(uint16_t color1, uint16_t color2);
unsigned long testRects(uint16_t color);
unsigned long testFilledRects(uint16_t color1, uint16_t color2);
unsigned long testFilledCircles(uint8_t radius, uint16_t color);
unsigned long testCircles(uint8_t radius, uint16_t color);
unsigned long testTriangles();
unsigned long testFilledTriangles();
unsigned long testRoundRects();
unsigned long testFilledRoundRects();
void testFillRoundRect();
void testRoundRect();
void testtriangles();
void testfilltriangles();
void testfillcircles(uint8_t radius, uint16_t color);
void testText(uint16_t color);

//---------------------------------------------------------------------------------------
void reportBenchMark()
{
	Serial.println(F("Benchmark                Time (microseconds)"));

	Serial.print(F("Text				"));
	Serial.println(testText());
	delay(3000);

	Serial.print(F("Lines				"));
	Serial.println(testLines(CYAN));
	delay(500);

	Serial.print(F("Horiz/Vert Lines		"));
	Serial.println(testFastLines(RED, BLUE));
	delay(500);

	Serial.print(F("Rectangles (outline)		"));
	Serial.println(testRects(GREEN));
	delay(500);

	Serial.print(F("Rectangles (filled)		"));
	Serial.println(testFilledRects(YELLOW, MAGENTA));
	delay(500);

	Serial.print(F("Circles (filled)		"));
	Serial.println(testFilledCircles(10, MAGENTA));

	Serial.print(F("Circles (outline)		"));
	Serial.println(testCircles(10, WHITE));
	delay(500);

	Serial.print(F("Triangles (outline)		"));
	Serial.println(testTriangles());
	delay(500);

	Serial.print(F("Triangles (filled)		"));
	Serial.println(testFilledTriangles());
	delay(500);

	Serial.print(F("Rounded rects (outline)		"));
	Serial.println(testRoundRects());
	delay(500);

	Serial.print(F("Rounded rects (filled)		"));
	Serial.println(testFilledRoundRects());
	delay(500);

	Serial.println(F("Done!"));
}

//---------------------------------------------------------------------------------------
unsigned long testText() {
	tft.fillScreen(BLACK);
	unsigned long start = micros();
	tft.setCursor(0, 0);
	tft.setTextColor(WHITE);
	tft.setTextSize(1);
	tft.println("Hello World!");
	tft.setTextColor(YELLOW);
	tft.setTextSize(2);
	tft.println(1234.56);
	tft.setTextColor(RED);
	tft.setTextSize(3);
	tft.println(0xDEADBEEF, HEX);
	tft.println();
	tft.setTextColor(GREEN);
	tft.setTextSize(5);
	tft.println("Groop");
	tft.setTextSize(2);
	tft.println("I implore thee,");
	tft.setTextSize(1);
	tft.println("my foonting turlingdromes.");
	tft.println("And hooptiously drangle me");
	tft.println("with crinkly bindlewurdles,");
	tft.println("Or I will rend thee");
	tft.println("in the gobberwarts");
	tft.println("with my blurglecruncheon,");
	tft.println("see if I don't!");
	return micros() - start;
}

//---------------------------------------------------------------------------------------
unsigned long testLines(uint16_t color) {
	unsigned long start, t;
	int x1, y1, x2, y2, w = tft.width(), h = tft.height();

	tft.fillScreen(BLACK);

	x1 = y1 = 0;
	y2 = h - 1;
	start = micros();
	for (x2 = 0; x2 < w; x2 += 6)
		tft.drawLine(x1, y1, x2, y2, color);
	x2 = w - 1;
	for (y2 = 0; y2 < h; y2 += 6)
		tft.drawLine(x1, y1, x2, y2, color);
	t = micros() - start; // fillScreen doesn't count against timing

	tft.fillScreen(BLACK);

	x1 = w - 1;
	y1 = 0;
	y2 = h - 1;
	start = micros();
	for (x2 = 0; x2 < w; x2 += 6)
		tft.drawLine(x1, y1, x2, y2, color);
	x2 = 0;
	for (y2 = 0; y2 < h; y2 += 6)
		tft.drawLine(x1, y1, x2, y2, color);
	t += micros() - start;

	tft.fillScreen(BLACK);

	x1 = 0;
	y1 = h - 1;
	y2 = 0;
	start = micros();
	for (x2 = 0; x2 < w; x2 += 6)
		tft.drawLine(x1, y1, x2, y2, color);
	x2 = w - 1;
	for (y2 = 0; y2 < h; y2 += 6)
		tft.drawLine(x1, y1, x2, y2, color);
	t += micros() - start;

	tft.fillScreen(BLACK);

	x1 = w - 1;
	y1 = h - 1;
	y2 = 0;
	start = micros();
	for (x2 = 0; x2 < w; x2 += 6)
		tft.drawLine(x1, y1, x2, y2, color);
	x2 = 0;
	for (y2 = 0; y2 < h; y2 += 6)
		tft.drawLine(x1, y1, x2, y2, color);
	return micros() - start;
}

//---------------------------------------------------------------------------------------
unsigned long testFastLines(uint16_t color1, uint16_t color2) {
	unsigned long start;
	int x, y, w = tft.width(), h = tft.height();

	tft.fillScreen(BLACK);
	start = micros();
	for (y = 0; y < h; y += 5)
		tft.drawFastHLine(0, y, w, color1);
	for (x = 0; x < w; x += 5)
		tft.drawFastVLine(x, 0, h, color2);
	return micros() - start;
}

//---------------------------------------------------------------------------------------
unsigned long testRects(uint16_t color) {
	unsigned long start;
	int n, i, i2, cx = tft.width() / 2, cy = tft.height() / 2;

	tft.fillScreen(BLACK);
	n = min(tft.width(), tft.height());
	start = micros();
	for (i = 2; i < n; i += 6)
	{
		i2 = i / 2;
		tft.drawRect(cx - i2, cy - i2, i, i, color);
	}
	return micros() - start;
}

//---------------------------------------------------------------------------------------
unsigned long testFilledRects(uint16_t color1, uint16_t color2) {
	unsigned long start, t = 0;
	int n, i, i2, cx = tft.width() / 2 - 1, cy = tft.height() / 2 - 1;

	tft.fillScreen(BLACK);
	n = min(tft.width(), tft.height());
	for (i = n; i > 0; i -= 6)
	{
		i2 = i / 2;
		start = micros();
		tft.fillRect(cx - i2, cy - i2, i, i, color1);
		t += micros() - start;
		// Outlines are not included in timing results
		tft.drawRect(cx - i2, cy - i2, i, i, color2);
	}
	return t;
}

//---------------------------------------------------------------------------------------
unsigned long testFilledCircles(uint8_t radius, uint16_t color) {
	unsigned long start;
	int x, y, w = tft.width(), h = tft.height(), r2 = radius * 2;

	tft.fillScreen(BLACK);
	start = micros();
	for (x = radius; x < w; x += r2)
	{
		for (y = radius; y < h; y += r2)
		{
			tft.fillCircle(x, y, radius, color);
		}
	}
	return micros() - start;
}

//---------------------------------------------------------------------------------------
unsigned long testCircles(uint8_t radius, uint16_t color) {
	unsigned long start;
	int x, y, r2 = radius * 2, w = tft.width() + radius, h = tft.height() + radius;

	// Screen is not cleared for this one -- this is
	// intentional and does not affect the reported time.
	start = micros();
	for (x = 0; x < w; x += r2)
	{
		for (y = 0; y < h; y += r2)
		{
			tft.drawCircle(x, y, radius, color);
		}
	}
	return micros() - start;
}

//---------------------------------------------------------------------------------------
unsigned long testTriangles() {
	unsigned long start;
	int n, i, cx = tft.width() / 2 - 1, cy = tft.height() / 2 - 1;

	tft.fillScreen(BLACK);
	n = min(cx, cy);
	start = micros();
	for (i = 0; i < n; i += 5)
	{
		tft.drawTriangle(cx, cy - i, // peak
		cx - i, cy + i, // bottom left
		cx + i, cy + i, // bottom right
		tft.color565(0, 0, i));
	}
	return micros() - start;
}

//---------------------------------------------------------------------------------------
unsigned long testFilledTriangles() {
	unsigned long start, t = 0;
	int i, cx = tft.width() / 2 - 1, cy = tft.height() / 2 - 1;

	tft.fillScreen(BLACK);
	start = micros();
	for (i = min(cx, cy); i > 10; i -= 5)
	{
		start = micros();
		tft.fillTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i, tft.color565(0, i, i));
		t += micros() - start;
		tft.drawTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i, tft.color565(i, i, 0));
	}
	return t;
}

//---------------------------------------------------------------------------------------
unsigned long testRoundRects() {
	unsigned long start;
	int w, i, i2, cx = tft.width() / 2 - 1, cy = tft.height() / 2 - 1;

	tft.fillScreen(BLACK);
	w = min(tft.width(), tft.height());
	start = micros();
	for (i = 0; i < w; i += 6)
	{
		i2 = i / 2;
		tft.drawRoundRect(cx - i2, cy - i2, i, i, i / 8, tft.color565(i, 0, 0));
	}
	return micros() - start;
}

//---------------------------------------------------------------------------------------
unsigned long testFilledRoundRects() {
	unsigned long start;
	int i, i2, cx = tft.width() / 2 - 1, cy = tft.height() / 2 - 1;

	tft.fillScreen(BLACK);
	start = micros();
	for (i = min(tft.width(), tft.height()); i > 20; i -= 6)
	{
		i2 = i / 2;
		tft.fillRoundRect(cx - i2, cy - i2, i, i, i / 8, tft.color565(0, i, 0));
	}
	return micros() - start;
}

//-----------------------------------------------------------------------------------------
void testFillRoundRect()
{
	tft.fillScreen(BLACK);
	for (uint16_t x = tft.width(); x > 20; x -= 6)
	{
		tft.fillRoundRect(tft.width() / 2 - x / 2, tft.height() / 2 - x / 2, x, x, x / 8,
				tft.Color565(x, x, 0));
	}
}

//---------------------------------------------------------------------------------------
void testRoundRect()
{
	tft.fillScreen(BLACK);
	for (int16_t x = 0; x < tft.width(); x += 6)
	{
		tft.drawRoundRect(tft.width() / 2 - x / 2, tft.height() / 2 - x / 2, x, x, x / 8,
				tft.Color565(x, 0, 0));
	}
}

//---------------------------------------------------------------------------------------
void testtriangles()
{
	tft.fillScreen(BLACK);
	for (int16_t i = 0; i < tft.width() / 2; i += 5)
	{
		tft.drawTriangle(tft.width() / 2, tft.height() / 2 - i, tft.width() / 2 - i,
				tft.height() / 2 + i, tft.width() / 2 + i, tft.height() / 2 + i,
				tft.Color565(0, 0, i));
	}
}

//---------------------------------------------------------------------------------------
void testfilltriangles()
{
	tft.fillScreen(BLACK);
	for (uint16_t i = tft.width() / 2; i > 10; i -= 5)
	{
		tft.fillTriangle(tft.width() / 2, tft.height() / 2 - i, tft.width() / 2 - i,
				tft.height() / 2 + i, tft.width() / 2 + i, tft.height() / 2 + i,
				tft.Color565(0, i, i));
		tft.drawTriangle(tft.width() / 2, tft.height() / 2 - i, tft.width() / 2 - i,
				tft.height() / 2 + i, tft.width() / 2 + i, tft.height() / 2 + i,
				tft.Color565(i, i, 0));
	}
}

//---------------------------------------------------------------------------------------
void testText(uint16_t color)
{
	tft.fillScreen(BLACK);
	tft.setCursor(5, 10);
	tft.setTextColor(color);
	tft.setTextSize(3);
	tft.println("Hello aitendo!");
	tft.setTextSize(3);
	tft.println(1234.56);
	tft.setTextSize(3);
	tft.println(0xDeadbeef, HEX);
}

//---------------------------------------------------------------------------------------
void testfillcircles(uint8_t radius, uint16_t color)
{
	tft.fillScreen(BLACK);
	for (int16_t x = radius; x < tft.width(); x += radius * 2)
	{
		//color = nextColor();
		for (int16_t y = radius; y < tft.height(); y += radius * 2)
		{
			tft.fillCircle(x, y, radius, nextColor());
		}
	}
}

//---------------------------------------------------------------------------------------
void testdrawcircles(uint8_t radius, uint16_t color)
{
	tft.fillScreen(BLACK);
	for (int16_t x = 0; x < tft.width() + radius; x += radius * 2)
	{
		for (int16_t y = 0; y < tft.height() + radius; y += radius * 2)
		{
			tft.drawCircle(x, y, radius, nextColor());
		}
	}
}

//---------------------------------------------------------------------------------------
void testfillrects(uint16_t color1, uint16_t color2)
{
	tft.fillScreen(BLACK);
	for (int16_t x = tft.width() - 1; x > 6; x -= 6)
	{
		//Serial.println(x, DEC);
		tft.fillRect(tft.width() / 2 - x / 2, tft.height() / 2 - x / 2, x, x, color1);
		tft.drawRect(tft.width() / 2 - x / 2, tft.height() / 2 - x / 2, x, x, color2);
	}
}

//---------------------------------------------------------------------------------------
void testdrawrects(uint16_t color)
{
	tft.fillScreen(BLACK);
	for (int16_t x = 0; x < tft.width(); x += 6)
	{
		tft.drawRect(tft.width() / 2 - x / 2, tft.height() / 2 - x / 2, x, x, color);
	}
}

//---------------------------------------------------------------------------------------
void testfastlines(uint16_t color1, uint16_t color2)
{
	tft.fillScreen(BLACK);
	for (int16_t y = 0; y < tft.height(); y += 5)
		tft.drawFastHLine(0, y, tft.width(), color1);
	for (int16_t x = 0; x < tft.width(); x += 5)
		tft.drawFastVLine(x, 0, tft.height(), color2);
}

//---------------------------------------------------------------------------------------
void testlines(uint16_t color)
{
	tft.fillScreen(BLACK);
	for (int16_t x = 0; x < tft.width(); x += 6)
		tft.drawLine(0, 0, x, tft.height() - 1, color);
	color = nextColor();
	for (int16_t y = 0; y < tft.height(); y += 6)
		tft.drawLine(0, 0, tft.width() - 1, y, color);

	tft.fillScreen(BLACK);
	color = nextColor();
	for (int16_t x = 0; x < tft.width(); x += 6)
		tft.drawLine(tft.width() - 1, 0, x, tft.height() - 1, color);
	color = nextColor();
	for (int16_t y = 0; y < tft.height(); y += 6)
		tft.drawLine(tft.width() - 1, 0, 0, y, color);

	tft.fillScreen(BLACK);
	color = nextColor();
	for (int16_t x = 0; x < tft.width(); x += 6)
		tft.drawLine(0, tft.height() - 1, x, 0, color);
	color = nextColor();
	for (int16_t y = 0; y < tft.height(); y += 6)
		tft.drawLine(0, tft.height() - 1, tft.width() - 1, y, color);

	tft.fillScreen(BLACK);
	color = nextColor();
	for (int16_t x = 0; x < tft.width(); x += 6)
		tft.drawLine(tft.width() - 1, tft.height() - 1, x, 0, color);
	color = nextColor();
	for (int16_t y = 0; y < tft.height(); y += 6)
		tft.drawLine(tft.width() - 1, tft.height() - 1, 0, y, color);
}

//---------------------------------------------------------------------------------------
void testBars()
{
	int16_t i, j;
	tft.drawLine(tft.width() - 1, tft.height() - 1, 0, 10, BLACK);
	for (i = 0; i < tft.height(); i++)
	{
		for (j = 0; j < tft.width(); j++)
		{
			if (i > 279)
				tft.writeData((uint16_t)WHITE);
			else if (i > 239)
				tft.writeData((uint16_t)BLUE);
			else if (i > 199)
				tft.writeData((uint16_t)GREEN);
			else if (i > 159)
				tft.writeData((uint16_t)CYAN);
			else if (i > 119)
				tft.writeData((uint16_t)RED);
			else if (i > 79)
				tft.writeData((uint16_t)MAGENTA);
			else if (i > 39)
				tft.writeData((uint16_t)YELLOW);
			else
				tft.writeData((uint16_t) BLACK);
		}
	}
}
//---------------------------------------------------------------------------------------
void testMe(short request)
{
	switch (request)
	{
	case 0:
		testdrawcircles(25, GetPenColor());
		break;
	case 1:
		testfillcircles(20, GetPenColor());
		break;
	case 2:
		testdrawrects(GetPenColor());
		break;
	case 3:
		testRoundRect();
		break;
	case 4:
		testfilltriangles();
		break;
	case 5:
		testtriangles();
		break;
	case 6:
		testBars();
		break;
	case 7:
		testFillRoundRect();
		break;
	}
}
#endif

#ifdef MEGA
//---------------------------------------------------------------------------------------
void myPingServiceTask()
{
	unsigned long start = micros();
	lastPingVal = ping();
	if(maxPingVal < lastPingVal)
		maxPingVal = lastPingVal;
	Serial.print("This Distance from ping is(Inches): ");
	Serial.print(lastPingVal,DEC);
	Serial.print(",");
	noInterrupts();
	serviceTicks = 50;
	interrupts();
	start = micros() - start;
	Serial.println(start, DEC);
}

//---------------------------------------------------------------------------------------
void myServoServiceTask()
{
	unsigned long start = micros();
	 // scale it to use it with the servo (value between 0 and 180)
	servoPos = map(lastPingVal, 0, maxPingVal, 0, 179);
	servo.write(servoPos);

	//Serial.print(servoPos, DEC);
	//Serial.print(" ");
	//Serial.print(lastPingVal, DEC);

	noInterrupts();
	serviceTicks1 = 5000;
	interrupts();
	start = micros() - start;
	//Serial.print(" ");
	//Serial.println(start, DEC);
}

//---------------------------------------------------------------------------------------
unsigned long ping()
{
	unsigned long echo = 0;
	unsigned long ultrasoundValue = 0;

	pinMode(USSPin, OUTPUT); // Switch signal pin to output
	digitalWrite(USSPin, LOW); // Send low pulse
	delayMicroseconds(2); // Wait for 2 microseconds
	digitalWrite(USSPin, HIGH); // Send high pulse
	delayMicroseconds(5); // Wait for 5 microseconds
	digitalWrite(USSPin, LOW); // Hold off
	pinMode(USSPin, INPUT); // Switch signal pin to input
	digitalWrite(USSPin, HIGH); // Turn on pull up resistor
	// please note that pulseIn has a 1sec timeout, which may not be desirable.
	// Depending on your sensor specifications, you can likely bound the time like this -- marcmerlin
	echo = pulseIn(USSPin, HIGH, 38000); //Listen for echo
	ultrasoundValue = (echo / 58.138) * 3.9; //convert to CM then to inches
	return ultrasoundValue;
}

//---------------------------------------------------------------------------------------
void ServiceDistanceSener()
{
	digitalWrite(TRIGPIN, LOW);
	delayMicroseconds(2);
	// Sets the trigPin on HIGH state for 10 micro seconds
	digitalWrite(TRIGPIN, HIGH);
	delayMicroseconds(8);
	digitalWrite(TRIGPIN, LOW);

	// Reads the echoPin, returns the sound wave travel time in microseconds
	double duration = pulseInLong(ECHOPIN, HIGH, 38000);

	// Calculating the distance
	//distance= duration*0.034/2;
	// Prints the distance on the Serial Monitor
	//char work[255];
	//char temp[12];

	double	distanceCms = (duration*0.034)/2.0;
	double	distanceInches = (duration*0.0133)/2.0;

	//memset(&work,0,255);
	//memset(&temp,0,12);
	//dtostrf(duration, 6, 2, work);
	//sprintf(work, "Duration: %s Distance: %d inches Distance in %d cms\n", temp, distanceInches, distanceCms);
	//Serial.print(work);
	Serial.print("Duration: ");
	Serial.print(duration);
	Serial.print(" Distance: ");
	Serial.print(distanceInches);
	Serial.print(" inches Distance: ");
	Serial.print(distanceCms);
	Serial.print(" cms");
	Serial.println();
	noInterrupts();
	serviceTicks = 60;
	interrupts();
}

#endif // end of MEGA


//---------------------------------------------------------------------------------------
#if 0
void testMain()
{
#define DELAYTIME (3300)
	delay(DELAYTIME);
	testFillRoundRect();
	delay(DELAYTIME);
	testRoundRect();
	delay(DELAYTIME);
	testtriangles();
	delay(DELAYTIME);
	testfilltriangles();
	delay(DELAYTIME);
	testText(nextColor());
	delay(DELAYTIME);
	testfillcircles(15, nextColor());
	delay(DELAYTIME);
	testdrawcircles(20, nextColor());
	delay(DELAYTIME);
	testfillrects(nextColor(), nextColor());
	delay(DELAYTIME);
	testdrawrects(nextColor());
	delay(DELAYTIME);
	testfastlines(nextColor(), nextColor());
	delay(DELAYTIME);
	testlines(nextColor());
	delay(DELAYTIME);
	testBars();
	delay(DELAYTIME);
}
//---------------------------------------------------------------------------------------
#endif
