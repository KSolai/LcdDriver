/*
 * Ping.cpp
 *
 *  Created on: Aug 3, 2017
 *      Author: Johnny
 */

#include "Ping.h"

Ping::Ping(): m_TrigPin(0), m_EchoPin(0), m_Bit(0), m_Port(0), m_StateMask(0),
			m_Sensor3Pin(false), m_Stage(INIT), m_Duration(0), m_Duration_(0)
{
	count=0;
}

Ping::~Ping() {
}

void Ping::Setup(uint8_t trigPin, uint8_t echoPin, uint8_t state)
{
	m_TrigPin = trigPin;
	m_EchoPin = echoPin;
	m_Sensor3Pin = m_TrigPin == m_EchoPin;
	m_Bit = digitalPinToBitMask(m_EchoPin);
	m_Port = digitalPinToPort(m_EchoPin);
	m_StateMask = (state ? m_Bit : 0);

	pinMode(m_TrigPin, OUTPUT);
	digitalWrite(m_TrigPin, LOW); // Send low pulse
	if(!m_Sensor3Pin)
	{
		pinMode(m_EchoPin, INPUT);
		digitalWrite(m_EchoPin, HIGH); // Set the pullup
	}
	m_Stage = START;
}
double Ping::GetDistanceInCms()
{
	return (0.0+m_Duration/58.0);
}
double Ping::GetDistanceInInches()
{
	return (0.0+m_Duration/148.0);
}
unsigned long Ping::GetDuration()
{
	return m_Duration;
}
void Ping::Service()
{
	//(count++ & 1)? digitalWrite(22, HIGH): digitalWrite(22, LOW);
	switch(m_Stage)
	{
	case	START:
		if(m_Sensor3Pin){
			pinMode(m_TrigPin, OUTPUT); // Set output mode
			digitalWrite(m_TrigPin, LOW); // Set to low
			delayMicroseconds(2); // Wait for 2 microseconds
		}
		digitalWrite(m_TrigPin, HIGH); // Set to high
		delayMicroseconds(10); // Wait for 5 microseconds
		digitalWrite(m_TrigPin, LOW); // Set back to low
		m_Stage = WAIT4HIGH;
		break;

	case	WAIT4HIGH:
		if((*portInputRegister(m_Port) & m_Bit) == m_StateMask ) {//|| ++m_Duration_ >8){
		//if(digitalRead(m_EchoPin) == 1 || ++m_Duration_ >8){
			m_Stage = WAIT4LOW;
			m_Duration_ = micros();
		}
		break;

	case	WAIT4LOW:
		if((*portInputRegister(m_Port) & m_Bit) != m_StateMask){
		//if(digitalRead(m_EchoPin) == 0){
			m_Stage = BREATH;
			m_Duration = micros() - m_Duration_;
			m_Duration_=0;
		}
		break;
	case	BREATH:
		if(++m_Duration_ > 50)
		{
			m_Stage = START;
			m_Duration_= 0;
		}
		break;
	case	INIT:
	default:
		break;;	// Do Nothing
	}
}
