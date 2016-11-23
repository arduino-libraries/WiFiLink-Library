/*
  serial.cpp - Library for Arduino Wifi shield.
  Copyright (c) 2011-2014 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
//*********************************************/
//
//  File:   serial.cpp
//
//  Author: bing@arduino.org (arduino srl)
//  edit: andrea@arduino.org (arduino srl)
//
//********************************************/

#if defined(__AVR_ATmega328P__)
  #include "SC16IS750.h"
#endif

#include "serial.h"

unsigned long _startMillis;
unsigned long _timeout = 3000; //3 Second Serial Timeout

void WfSerial::begin()
{

// #if defined(__AVR_ATmega328P__)
//   ESPSerial.begin(9600);
#if defined(ARDUINO_PRIMO) || defined(__AVR_ATmega32U4__)  //to test
  Serial1.begin(9600);
#else
//   Serial.begin(9600);
 #endif

}

int WfSerial::read()
{
  int c;

#if defined(__AVR_ATmega328P__)
  c = ESPSerial.read();
#elif defined(ARDUINO_PRIMO) || defined(__AVR_ATmega32U4__)  //added to test with arduino Leonardo
  c = Serial1.read();
#else
  c = Serial.read();
#endif

  return c;

}

void WfSerial::write(unsigned char c)
{

#if defined(__AVR_ATmega328P__)
  ESPSerial.write(c);
#elif defined(ARDUINO_PRIMO) || defined(__AVR_ATmega32U4__)
  Serial1.write(c);
#else
  Serial.write(c);
#endif

}

int WfSerial::available()
{

  int num;

#if defined(__AVR_ATmega328P__)
  num = ESPSerial.available();
#elif defined(ARDUINO_PRIMO) || defined(__AVR_ATmega32U4__)
  num = Serial1.available();
#else
  num = Serial.available();
#endif

  return num;

}

int WfSerial::timedRead()
{
  int c;
  _startMillis = millis();
  do {
    //c = Serial1.read();//
    c = read();
    if (c >= 0) return c;
  } while(millis() - _startMillis < _timeout);
  return -1;     // -1 indicates timeout
}

String WfSerial::readStringUntil(char terminator){

	String ret;
	int c = timedRead();

	while (c >= 0 && (char)c != terminator)
	{
		ret += (char)c;
		c = timedRead();
	}
	return ret;

}

WfSerial wfSerial;
