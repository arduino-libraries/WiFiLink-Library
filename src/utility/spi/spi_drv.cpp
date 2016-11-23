/*
  spi_drv.cpp - Library for Arduino Wifi shield.
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

#include "Arduino.h"
#include <SPI.h>
#include "spi_drv.h"
#include "pins_arduino.h"
//#define _DEBUG_
extern "C" {
#include "utility/debug.h"
}

#define DATAOUT     11 // MOSI
#define DATAIN      12 // MISO
#define SPICLOCK    13 // sck
#define SLAVESELECT 10 // ss
#define SLAVEREADY  7  // handshake pin
#define WIFILED     9  // led on wifi shield

#define DELAY_SPI(X) { int ii=0; do { asm volatile("nop"); } while (++ii < (X*F_CPU/16000000)); }
#define DELAY_TRANSFER() DELAY_SPI(10)

int byte_transfer = 0;

void SpiDrv::begin()
{
	  SPI.begin();
	  pinMode(SLAVESELECT, OUTPUT);
	  pinMode(SLAVEREADY, INPUT);
	  pinMode(WIFILED, OUTPUT);

	  digitalWrite(SCK, LOW);
	  digitalWrite(MOSI, LOW);
	  digitalWrite(SS, HIGH);
	  digitalWrite(SLAVESELECT, HIGH);
	  digitalWrite(WIFILED, LOW);

#ifdef _DEBUG_
	  INIT_TRIGGER()
#endif
}

void SpiDrv::end() {
    SPI.end();
}

void SpiDrv::commSlaveSelect()
{
    digitalWrite(SLAVESELECT,LOW);
}


void SpiDrv::commSlaveDeselect()
{
    digitalWrite(SLAVESELECT,HIGH);
}


char SpiDrv::commTransfer(volatile char data)
{
    char result = SPI.transfer(data);
		//Serial.print(result,HEX);
		byte_transfer++;
    DELAY_TRANSFER();

    return result;                    // return the received byte
}
int SpiDrv::readStatus()
{
		SPI.transfer(0x04);
		uint32_t status = (SPI.transfer(0) | ((uint32_t)(SPI.transfer(0)) << 8) | ((uint32_t)(SPI.transfer(0)) << 16) | ((uint32_t)(SPI.transfer(0)) << 24));
		return status;
}
void SpiDrv::writeStatus(uint32_t status)
{
		//digitalWrite(_ss_pin, LOW);
		//delayMicroseconds(10);
		SPI.transfer(0x01);
		SPI.transfer(status & 0xFF);
		SPI.transfer((status >> 8) & 0xFF);
		SPI.transfer((status >> 16) & 0xFF);
		SPI.transfer((status >> 24) & 0xFF);
		//digitalWrite(_ss_pin, HIGH);
}

int SpiDrv::waitCommChar(unsigned char waitChar)
{
    int timeout = TIMEOUT_CHAR;
    unsigned char _readChar = 0;
    do{
        _readChar = readChar(); //get data byte
        if (_readChar == ERR_CMD)
        {
        	WARN("Err cmd received\n");
        	return -1;
        }
    }while((timeout-- > 0) && (_readChar != waitChar));
    return  (_readChar == waitChar);
}

int SpiDrv::readAndCheckChar(char checkChar, char* readChar)
{
    getParam((uint8_t*)readChar);

    return  (*readChar == checkChar);
}

char SpiDrv::readChar()
{
	uint8_t readChar = 0;
	getParam(&readChar);
	return readChar;
}

#define WAIT_START_CMD(x) waitCommChar(0xE0)

#define IF_CHECK_START_CMD(x)                      \
    if (!WAIT_START_CMD(_data))                 \
    {                                           \
        TOGGLE_TRIGGER()                        \
        WARN("Error waiting START_CMD");        \
        return 0;                               \
    }else                                       \

#define CHECK_DATA(check, x)                   \
        if (!readAndCheckChar(check, &x))   \
        {                                               \
        	TOGGLE_TRIGGER()                        \
            WARN("Reply error");                        \
            INFO2(check, (uint8_t)x);							\
            return 0;                                   \
        }else                                           \

#define waitSlaveReady() (digitalRead(SLAVEREADY) == LOW)
#define waitSlaveSign() (digitalRead(SLAVEREADY) == HIGH)
#define waitSlaveSignalH() while(digitalRead(SLAVEREADY) != HIGH){}
#define waitSlaveSignalL() while(digitalRead(SLAVEREADY) != LOW){}

void SpiDrv::waitForSlaveSign()
{
	while (!waitSlaveSign());
}

void SpiDrv::waitForSlaveReady()
{
	commSlaveDeselect();
	commSlaveSelect();
	writeStatus(1);
	commSlaveDeselect();
	delayMicroseconds(SLAVEREADY_TIME);
	commSlaveSelect();

	//while
	// int loop =0;
	// while(loop <100){
	// 	int rs = readStatus();
	// 	if(rs == 2)
	// 		break;
	// 	delayMicroseconds(10);
	// 	loop++;
	// }
}

void SpiDrv::getParam(uint8_t* param)
{
    // Get Params data
    *param = commTransfer(DUMMY_DATA);
    DELAY_TRANSFER();
}

int SpiDrv::waitResponseCmd(uint8_t cmd, uint8_t numParam, uint8_t* param, uint8_t* param_len)
{
    char _data = 0;
    int ii = 0;
		SPI.transfer(0x03);
		SPI.transfer(0x00);

    IF_CHECK_START_CMD(_data)
    {
				//Serial.println("0");
        CHECK_DATA(cmd | REPLY_FLAG, _data){};
				//Serial.println("1");
        CHECK_DATA(numParam, _data);
        {
						//Serial.println("2");
            readParamLen8(param_len);
						//Serial.println("3");
            for (ii=0; ii<(*param_len); ++ii)
            {
                // Get Params data
                //param[ii] = commTransfer(DUMMY_DATA);
								//Serial.println("4");
								checkReceiverPacket();
                getParam(&param[ii]);
								//Serial.println("5");
								//Serial.print(param[ii],HEX);
            }
        }
				//Serial.println("6");
				checkReceiverPacket();
        readAndCheckChar(END_CMD, &_data);
				//Serial.println("7");
				//Serial.print("WaitResponseCmd: ");
				endPacket();

    }
    return 1;
}
/*
int SpiDrv::waitResponse(uint8_t cmd, uint8_t numParam, uint8_t* param, uint16_t* param_len)
{
    char _data = 0;
    int i =0, ii = 0;

    IF_CHECK_START_CMD(_data)
    {
        CHECK_DATA(cmd | REPLY_FLAG, _data){};

        CHECK_DATA(numParam, _data);
        {
            readParamLen16(param_len);
            for (ii=0; ii<(*param_len); ++ii)
            {
                // Get Params data
                param[ii] = commTransfer(DUMMY_DATA);
            }
        }

        readAndCheckChar(END_CMD, &_data);
    }

    return 1;
}
*/

int SpiDrv::waitResponseData16(uint8_t cmd, uint8_t* param, uint16_t* param_len)
{
    char _data = 0;
    uint16_t ii = 0;
    IF_CHECK_START_CMD(_data)
    {
        CHECK_DATA(cmd | REPLY_FLAG, _data){};

        uint8_t numParam = readChar();
        if (numParam != 0)
        {
            readParamLen16(param_len);
            for (ii=0; ii<(*param_len); ++ii)
            {
                // Get Params data
                param[ii] = commTransfer(DUMMY_DATA);
            }
        }

        readAndCheckChar(END_CMD, &_data);
    }

    return 1;
}

int SpiDrv::waitResponseData8_debug(uint8_t cmd, uint8_t* param, uint8_t* param_len)
{
    char _data = 0;
    int ii = 0;
		SPI.transfer(0x03);
		SPI.transfer(0x00);
		Serial.println("0");
    IF_CHECK_START_CMD(_data)
    {
			Serial.println("1");
        CHECK_DATA(cmd | REPLY_FLAG, _data){};
				Serial.println("2");
        uint8_t numParam = readChar();
        if (numParam != 0)
        {
					Serial.println("3");
            readParamLen8(param_len);
            for (ii=0; ii<(*param_len); ++ii)
            {
                // Get Params data
								Serial.println("4");
								checkReceiverPacket();
								Serial.println("5");
                param[ii] = commTransfer(DUMMY_DATA);
								Serial.println("6");
            }
        }
				Serial.println("7");
				checkReceiverPacket();
				Serial.println("8");
        readAndCheckChar(END_CMD, &_data);
				Serial.println("9");
				endPacket();
    }

    return 1;
}

int SpiDrv::waitResponseData8(uint8_t cmd, uint8_t* param, uint8_t* param_len)
{
    char _data = 0;
    int ii = 0;
		SPI.transfer(0x03);
		SPI.transfer(0x00);
    IF_CHECK_START_CMD(_data)
    {
        CHECK_DATA(cmd | REPLY_FLAG, _data){};

        uint8_t numParam = readChar();
        if (numParam != 0)
        {
            readParamLen8(param_len);
            for (ii=0; ii<(*param_len); ++ii)
            {
                // Get Params data
								checkReceiverPacket();
                param[ii] = commTransfer(DUMMY_DATA);
            }
        }

				checkReceiverPacket();
        readAndCheckChar(END_CMD, &_data);
				endPacket();
    }

    return 1;
}

int SpiDrv::waitResponseParams(uint8_t cmd, uint8_t numParam, tParam* params)
{
    char _data = 0;
    int i =0, ii = 0;
		SPI.transfer(0x03);
		SPI.transfer(0x00);

    IF_CHECK_START_CMD(_data)
    {
        CHECK_DATA(cmd | REPLY_FLAG, _data){};

        uint8_t _numParam = readChar();
        if (_numParam != 0)
        {
            for (i=0; i<_numParam; ++i)
            {
                params[i].paramLen = readParamLen8();
                for (ii=0; ii<params[i].paramLen; ++ii)
                {
                    // Get Params data
                    params[i].param[ii] = commTransfer(DUMMY_DATA);
                }
            }
        } else
        {
            WARN("Error numParam == 0");
            return 0;
        }

        if (numParam != _numParam)
        {
            WARN("Mismatch numParam");
            return 0;
        }

        readAndCheckChar(END_CMD, &_data);
    }
    return 1;
}

/*
int SpiDrv::waitResponse(uint8_t cmd, tParam* params, uint8_t* numParamRead, uint8_t maxNumParams)
{
    char _data = 0;
    int i =0, ii = 0;

    IF_CHECK_START_CMD(_data)
    {
        CHECK_DATA(cmd | REPLY_FLAG, _data){};

        uint8_t numParam = readChar();

        if (numParam > maxNumParams)
        {
            numParam = maxNumParams;
        }
        *numParamRead = numParam;
        if (numParam != 0)
        {
            for (i=0; i<numParam; ++i)
            {
                params[i].paramLen = readParamLen8();

                for (ii=0; ii<params[i].paramLen; ++ii)
                {
                    // Get Params data
                    params[i].param[ii] = commTransfer(DUMMY_DATA);
                }
            }
        } else
        {
            WARN("Error numParams == 0");
            Serial.println(cmd, 16);
            return 0;
        }
        readAndCheckChar(END_CMD, &_data);
    }
    return 1;
}
*/

int SpiDrv::waitResponse(uint8_t cmd, uint8_t* numParamRead, uint8_t** params, uint8_t maxNumParams)
{
    char _data = 0;
    int i =0, ii = 0;

    char    *index[WL_SSID_MAX_LENGTH];

    for (i = 0 ; i < WL_NETWORKS_LIST_MAXNUM ; i++)
            index[i] = (char *)params + WL_SSID_MAX_LENGTH*i;

    IF_CHECK_START_CMD(_data)
    {
        CHECK_DATA(cmd | REPLY_FLAG, _data){};

        uint8_t numParam = readChar();

        if (numParam > maxNumParams)
        {
            numParam = maxNumParams;
        }
        *numParamRead = numParam;
        if (numParam != 0)
        {
            for (i=0; i<numParam; ++i)
            {
            	uint8_t paramLen = readParamLen8();
                for (ii=0; ii<paramLen; ++ii)
                {
                	//ssid[ii] = commTransfer(DUMMY_DATA);
                    // Get Params data
                    index[i][ii] = (uint8_t)commTransfer(DUMMY_DATA);

                }
                index[i][ii]=0;
            }
        } else
        {
            WARN("Error numParams == 0");
            readAndCheckChar(END_CMD, &_data);
            return 0;
        }
        readAndCheckChar(END_CMD, &_data);
				//Serial.print("WaitResponse: ");
				endPacket();
    }
    return 1;
}


void SpiDrv::sendParam(uint8_t* param, uint8_t param_len, uint8_t lastParam)
{
    int i = 0;
		
    // Send Spi paramLen
    sendParamLen8(param_len);
    // Send Spi param data
    for (i=0; i<param_len; ++i)
    {

			checkTransferPacket();
      commTransfer(param[i]);
    }
    // if lastParam==1 Send Spi END CMD
    if (lastParam == 1){
				checkTransferPacket();
        commTransfer(END_CMD);
				//Serial.print("sendParam: ");
				endPacket();
		}

}

void SpiDrv::checkTransferPacket(){
	if(byte_transfer == SPI_SLAVE_BUFFER){
		byte_transfer = 0;
		delayMicroseconds(200);
		SPI.transfer(0x02);
		SPI.transfer(0x00);
	}
}

void SpiDrv::checkReceiverPacket(){
	if(byte_transfer == SPI_SLAVE_BUFFER){
		byte_transfer = 0;
		delayMicroseconds(200);
		SPI.transfer(0x03);
		SPI.transfer(0x00);
	}
}

void SpiDrv::endPacket(){

		for(int i=byte_transfer;i<32;i++)
				SPI.transfer(0);
		//Serial.println(byte_transfer);
		byte_transfer = 0;
}

void SpiDrv::sendParamLen8(uint8_t param_len)
{
    // Send Spi paramLen
    commTransfer(param_len);
		//byte_transfer = byte_transfer +1;
}

void SpiDrv::sendParamLen16(uint16_t param_len)
{
    // Send Spi paramLen
    commTransfer((uint8_t)((param_len & 0xff00)>>8));
    commTransfer((uint8_t)(param_len & 0xff));
		//byte_transfer = byte_transfer + 2;
}

uint8_t SpiDrv::readParamLen8(uint8_t* param_len)
{
    uint8_t _param_len = commTransfer(DUMMY_DATA);
    if (param_len != NULL)
    {
        *param_len = _param_len;
    }
    return _param_len;
}

uint16_t SpiDrv::readParamLen16(uint16_t* param_len)
{
    uint16_t _param_len = commTransfer(DUMMY_DATA)<<8 | (commTransfer(DUMMY_DATA)& 0xff);
    if (param_len != NULL)
    {
        *param_len = _param_len;
    }
    return _param_len;
}


void SpiDrv::sendBuffer(uint8_t* param, uint16_t param_len, uint8_t lastParam)
{
    uint16_t i = 0;

    // Send Spi paramLen
    sendParamLen16(param_len);

    // Send Spi param data
    for (i=0; i<param_len; ++i)
    {
				checkTransferPacket();
        commTransfer(param[i]);
    }

    // if lastParam==1 Send Spi END CMD
    if (lastParam == 1){
				checkTransferPacket();
        commTransfer(END_CMD);
				//Serial.print("sendBuffer: ");
				endPacket();

		}
}


void SpiDrv::sendParam(uint16_t param, uint8_t lastParam)
{
	// Send Spi paramLen
	sendParamLen8(2);
	//Serial.println("entrato in 2 param");
	//commTransfer(0x30);
	commTransfer((uint8_t)((param & 0xff00)>>8));
	commTransfer((uint8_t)(param & 0xff));
 //commTransfer(0x00);
 //commTransfer(0x50);
	//byte_transfer = byte_transfer + 2;
	// if lastParam==1 Send Spi END CMD
	if (lastParam == 1){
			commTransfer(END_CMD);
			//Serial.print("sendParam: ");
			endPacket();
			//byte_transfer = byte_transfer + 3;
	}

}

/* Cmd Struct Message */
/* _________________________________________________________________________________  */
/*| START CMD | C/R  | CMD  |[TOT LEN]| N.PARAM | PARAM LEN | PARAM  | .. | END CMD | */
/*|___________|______|______|_________|_________|___________|________|____|_________| */
/*|   8 bit   | 1bit | 7bit |  8bit   |  8bit   |   8bit    | nbytes | .. |   8bit  | */
/*|___________|______|______|_________|_________|___________|________|____|_________| */

void SpiDrv::sendCmd(uint8_t cmd, uint8_t numParam)
{
    // Send Spi START CMD

		SPI.transfer(0x02);		//need to start communication
		SPI.transfer(0x00);
		byte_transfer = 0;
		// Send Spi START CMD
    commTransfer(START_CMD);

    //waitForSlaveSign();
    //wait the interrupt trigger on slave
    delayMicroseconds(SPI_START_CMD_DELAY);

    // Send Spi C + cmd
    commTransfer(cmd & ~(REPLY_FLAG));

    // Send Spi numParam
    commTransfer(numParam);

		//byte_transfer = 3;
    // If numParam == 0 send END CMD
    if (numParam == 0){
        commTransfer(END_CMD);
				//Serial.print("send cmd: ");
				endPacket();
		}

}
