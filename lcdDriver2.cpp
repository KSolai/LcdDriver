/*
 * lcdDriver2.cpp
 *
 *  Created on: Jan 31, 2016
 *      Author: Kumar
 */
#include "lcdDriver2.h"

LcdDriver::LcdDriver(uint8_t cs, uint8_t cd, uint8_t wr, uint8_t rd, uint8_t reset)
{
	driver = 0;
	_reset = reset;
	pinMode(cs, OUTPUT);    // Enable outputs
	pinMode(cd, OUTPUT);
	pinMode(wr, OUTPUT);
	pinMode(rd, OUTPUT);
	if (reset)
	{
		digitalWrite(reset, HIGH);
		pinMode(reset, OUTPUT);
	}
}

void LcdDriver::init()
{
	setWrDirection(false);
}

//-----------------------------------------------------------------------------
uint16_t LcdDriver::readRegister(uint16_t addr)
{
	writeCommand(addr);
	return readData();
}
//-----------------------------------------------------------------------------
uint32_t LcdDriver::readRegister32(uint8_t addr)
{
	writeCommand(addr);
	return readData32();
}
//-----------------------------------------------------------------------------
uint8_t LcdDriver::read_Data()
{
	uint8_t data;
	CTRLPORT &= ~RD_MASK;
	DELAY7;//delayMicroseconds(0.4);
#ifdef UNO
	data = (PIND&DATA1_MASK)|(PINB&DATA2_MASK);
#endif
#ifdef MEGA
//	data = ((PINH&0x18)<<3) | ((PINB&0xB0)>>2) | ((PING&0x20)>>1) | ((PINH&0x60)>>5);
	data = ((PINH&0x60)>>5)|((PINH&0x18)<<3)|((PINE&0x08)<<2)|((PINE&0x30)>>2)|((PING&0x20)>>1);
#endif
	CTRLPORT |= RD_MASK;
	return data;
}
//-----------------------------------------------------------------------------
uint32_t LcdDriver::readData32()
{
	uint32_t result = 0;
	uint8_t* pData = (uint8_t *) &result;
	setRdDirection();
	*(pData + 3) = read_Data();
	*(pData + 2) = read_Data();
	*(pData + 1) = read_Data();
	*(pData + 0) = read_Data();
	delayMicroseconds(10);
	resetDirection();
	Serial.println(result, HEX);
	//setWrDirection(false);
	return result;
}
//-----------------------------------------------------------------------------
//--- Sets the direction for command(command==true) or data(command==false)
void LcdDriver::setWrDirection(bool command)
{
#ifdef UNO
	DATADDR1 |= DATA1_MASK;
	DATADDR2 |= DATA2_MASK;
#endif
#ifdef MEGA
//	DATADDR3 |= DATA3_MASK;
DDRH |= 0x78;
DDRE |= 0x38;
DDRG |= 0x20;
#endif
	CTRLPORT = command ? CTRLWRCMDMASK : CTRLWRMASK;
}
//-----------------------------------------------------------------------------
void LcdDriver::setRdDirection(bool command)
{
#ifdef UNO
	DATADDR1 &= ~DATA1_MASK;
	DATADDR2 &= ~DATA2_MASK;
#endif
#ifdef MEGA
//	DATADDR3 &= ~DATA3_MASK;
  DDRH &= ~0x78;
  DDRE &= ~0x38;
  DDRG &= ~0x20;
#endif
  CTRLPORT = command ? CTRLWRCMDMASK : CTRLWRMASK & ~RD_MASK;
}
//-----------------------------------------------------------------------------
void LcdDriver::writeData(uint8_t * data)
{
#ifdef UNO
	DATAPORT1 = (DATAPORT1 & DATA2_MASK) | (*data & DATA1_MASK); // top 6 bits
	DATAPORT2 = (DATAPORT2 & DATA1_MASK) | (*data & DATA2_MASK); // bottom 2 bits
#endif
#ifdef MEGA
	uint8_t d = * data;
    PORTH = (PORTH&~0x78) | ((d&0xC0)>>3) | ((d&0x03)<<5);
    PORTE = (PORTE&~0x38) | ((d&0x0C)<<2) | ((d&0x20)>>2);
    PORTG = (PORTG&~0x20) | ((d&0x10)<<1);
//	DATAPORT1 = (DATAPORT1 & DATA1_MASK) | (((d&0xC0)>>3)) | (((d&0x03)<<5) & DATA1_MASK);
//	DATAPORT2 = (DATAPORT2 & DATA2_MASK) | (((d&0x2C)<<2)) | (((d&0x20)>>2) & DATA2_MASK);
//	DATAPORT3 = (DATAPORT3 & DATA3_MASK) | (((d&0x10)<<1)  & DATA3_MASK);
#endif
	// write strobe
    CTRLPORT &= ~WR_MASK;
    DELAY7;
    CTRLPORT |= WR_MASK;
}
//-----------------------------------------------------------------------------
void LcdDriver::writeData(uint8_t data)
{
	setWrDirection(false);
	writeData( &data);
	resetDirection();
}
//-----------------------------------------------------------------------------
void LcdDriver::writeData(uint16_t data)
{
	uint8_t* pData = (uint8_t *) &data;
	setWrDirection(false);
	writeData(pData + 1);
	writeData(pData);
	resetDirection();
}
//-----------------------------------------------------------------------------
void LcdDriver::writeData(uint32_t data, bool is24)
{
	uint8_t* pData = (uint8_t *) &data;
	setWrDirection(false);
	if(!is24)
		writeData(pData + 3);
	writeData(pData + 2);
	writeData(pData + 1);
	writeData(pData);
	resetDirection();
}
//-----------------------------------------------------------------------------
void LcdDriver::writeCommand(uint8_t command)
{
	setWrDirection(true);
	writeData(&command);
	resetDirection();
}
//-----------------------------------------------------------------------------
void LcdDriver::writeCommand(uint16_t command)
{
	uint8_t* pCmd = (uint8_t *) &command;
	setWrDirection(true);
	writeData(pCmd + 1);
	writeData(pCmd);
	resetDirection();
}
//-----------------------------------------------------------------------------
void LcdDriver::writeRegister8(uint8_t addr, uint8_t data)
{
	writeCommand(addr);
	writeData(data);
}
//-----------------------------------------------------------------------------
void LcdDriver::writeRegister16(uint16_t addr, uint16_t data)
{
	writeCommand(addr);
	writeData(data);
}
//-----------------------------------------------------------------------------
void LcdDriver::writeRegister24(uint8_t addr, uint32_t data)
{
	writeCommand(addr);
	writeData(data, true);
}

//-----------------------------------------------------------------------------
uint16_t LcdDriver::readData()
{
	uint16_t result = 0;
	uint8_t* pData = (uint8_t *) &result;
	setRdDirection();
  *(pData + 1) = read_Data();
  *(pData + 0) = read_Data();
	resetDirection();
	return result;
}

//-----------------------------------------------------------------------------
void LcdDriver::writeRegisterPair(uint8_t aH, uint8_t aL, uint16_t d)
{
	uint8_t hi = d >> 8, lo = d;
	writeCommand(aH);
	writeData(hi);
	writeCommand(aL);
	writeData(lo);
}

//-----------------------------------------------------------------------------
void LcdDriver::writeRegister32(uint8_t addr, uint32_t data)
{
	writeCommand(addr);
	writeData(data);
}
//-----------------------------------------------------------------------------
