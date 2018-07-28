/*
 * Ping.h
 *
 *  Created on: Aug 3, 2017
 *      Author: Johnny
 */

#ifndef PING_H_
#define PING_H_

#if ARDUINO >= 100
#include "Arduino.h"
#include "Print.h"
#else
#include "WProgram.h"
#endif

enum stage{
	INIT,
	START,
	WAIT4HIGH,
	WAIT4LOW,
	BREATH
};
class Ping {
public:
	Ping();
	void Setup(uint8_t trigPin, uint8_t echoPin, uint8_t state);
	void Service();
	virtual ~Ping();
	double GetDistanceInCms();
	double GetDistanceInInches();
	unsigned long GetDuration();

private:
	uint8_t m_TrigPin;
	uint8_t m_EchoPin;
	uint8_t m_Bit;
	uint8_t m_Port;
	uint8_t m_StateMask;
	bool    m_Sensor3Pin;
	stage 	m_Stage;
	unsigned long	m_Duration;
	unsigned long	m_Duration_;
	unsigned long 	count;
};

#endif /* PING_H_ */
