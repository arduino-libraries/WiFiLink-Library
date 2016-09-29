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
// 
//********************************************/

#if defined(ARDUINO_UNO_WIFI)
  #include "SC16IS750.h"
#endif

#include "serial.h"
#include "Arduino.h"
  
void WfSerial::begin()
{

#if defined(ARDUINO_UNO_WIFI) 
  ESPSerial.begin(9600);
#elif defined(ARDUINO_PRIMO)
  Serial1.begin(9600);
#else
  Serial.begin(9600);
#endif

}
    
unsigned char WfSerial::read()
{
  unsigned char c;

#if defined(ARDUINO_UNO_WIFI) 
  c = ESPSerial.read();
#elif defined(ARDUINO_PRIMO)
  c = Serial1.read();
#else
  c = Serial.read();
#endif
  
  return c;

}

void WfSerial::write(unsigned char c)
{

#if defined(ARDUINO_UNO_WIFI) 
  ESPSerial.write(c);
#elif defined(ARDUINO_PRIMO)
  Serial1.write(c);
#else
  Serial.write(c);
#endif

}

int WfSerial::available()
{

  int num;

#if defined(ARDUINO_UNO_WIFI) 
  num = ESPSerial.available();
#elif defined(ARDUINO_PRIMO)
  num = Serial1.available();
#else
  num = Serial.available();
#endif

  return num;

}

WfSerial wfSerial;
