/*
 * lcdDriver2.h
 *
 *  Created on: Jan 31, 2016
 *      Author: Kumar
 */

#ifndef LCDDRIVER2_H_
#define LCDDRIVER2_H_

#if ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#if defined(__SAM3X8E__)
#include <include/pio.h>
#define PROGMEM
#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#define pgm_read_word(addr) (*(const unsigned short *)(addr))
#endif
#ifdef __AVR__
#include <avr/pgmspace.h>
#endif

#include <pins_arduino.h>
#include <wiring_private.h>
// **** IF USING THE LCD BREAKOUT BOARD, COMMENT OUT THIS NEXT LINE. ****
// **** IF USING THE LCD SHIELD, LEAVE THE LINE ENABLED:             ****
#define USE_ADAFRUIT_SHIELD_PINOUT

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__) || defined (__AVR_ATmega328__) || defined(__AVR_ATmega8__)
#define	UNO
#elif defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2561__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
#define MEGA
#include <avr/iom2560.h>
#endif

#ifndef DELAY7
#define DELAY7        \
  asm volatile(       \
    "rjmp .+0" "\n\t" \
    "rjmp .+0" "\n\t" \
    "rjmp .+0" "\n\t" \
    "nop"      "\n"   \
    ::);
#endif
// LCD controller chip identifiers
#define ID_932X    0
#define ID_7575    1
#define ID_9341    2
#define ID_HX8357D 3
#define ID_4535    4
#define ID_C505    5
#define ID_UNKNOWN 0xFF

/************************************ Kumar's own setting *****************************************/
#ifdef UNO
#define CTRLPORT PORTC    //Control Port
#define CTRLDIR DDRC      //Control pin direction register
#endif
#ifdef __AVR_ATmega2560__
#define CTRLPORT PORTF
#define CTRLDIR DDRF      //Control pin direction register
#endif

#define RD_MASK B00000001 //Read A0
#define WR_MASK B00000010 //Write A1
#define CD_MASK B00000100 //Command/Data A2
#define CS_MASK B00001000 //Chip select A3
#define RS_MASK B00010000 //Reset A4

#define CTRLRSETMASK	(RS_MASK | CS_MASK | CD_MASK | WR_MASK | RD_MASK) //Normal mask
#define CTRLWRMASK		(RS_MASK | CD_MASK | WR_MASK | RD_MASK)	//except the chip select
#define CTRLWRCMDMASK	(RS_MASK | WR_MASK | RD_MASK)		//except the CS and CD

#ifdef UNO
//Write data
#define DATAPORT1 PORTD
#define DATAPORT2 PORTB
//Common mask
#define DATA1_MASK 0xFC  // top 6 bits
#define DATA2_MASK 0x03  // bottom 2 bits
//Read data
#define DATAPIN1  PIND
#define DATAPIN2  PINB
//Direction setting
#define DATADDR1 DDRD
#define DATADDR2 DDRB
#endif

class LcdDriver {

public:
	LcdDriver(uint8_t cs, uint8_t cd, uint8_t wr, uint8_t rd, uint8_t reset);
	LcdDriver();

	~LcdDriver(){};

	void init();
private:
	void writeData(uint8_t  * data);
	uint8_t read_Data();
	uint32_t readData32();

public:
	uint16_t readData();
	uint16_t readRegister(uint16_t addr);
	uint32_t readRegister32(uint8_t addr);

	void writeData(uint8_t data);
	void writeData(uint16_t data);
	void writeData(uint32_t data, bool is24=false);
	void setWrDirection(bool command);
	void setRdDirection(bool command=false);
	void resetDirection(){ CTRLPORT = CTRLRSETMASK; }


	void writeCommand(uint8_t command);
	void writeRegister8(uint8_t addr, uint8_t data);
	void writeCommand(uint16_t command);
	void writeRegister16(uint16_t addr, uint16_t data);

	void writeRegister24(uint8_t a, uint32_t d);
	void writeRegister32(uint8_t a, uint32_t d);
	void writeRegisterPair(uint8_t aH, uint8_t aL, uint16_t d);

#ifndef USE_ADAFRUIT_SHIELD_PINOUT

#ifdef __AVR__1
//	volatile uint8_t *csPort, *cdPort, *wrPort, *rdPort;
//	uint8_t csPinSet, cdPinSet, wrPinSet, rdPinSet, csPinUnset, cdPinUnset, wrPinUnset, rdPinUnset,
//			_reset;
#endif
#if defined(__SAM3X8E__)
	Pio *csPort , *cdPort , *wrPort , *rdPort;
	uint32_t csPinSet , cdPinSet , wrPinSet , rdPinSet ,
	csPinUnset, cdPinUnset, wrPinUnset, rdPinUnset,
	_reset;
#endif

#endif
	uint8_t _reset;
	uint8_t driver;
};

#endif /* LCDDRIVER2_H_ */
