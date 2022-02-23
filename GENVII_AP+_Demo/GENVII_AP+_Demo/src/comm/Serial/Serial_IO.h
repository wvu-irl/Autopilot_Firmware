/*
 * Serial_IO.h
 *
 *  Created on: Mar 26, 2015
 *      Author: Matt Gramlich
 *      Update: Modified for GENVII Autopilot, Scott Harper 8/2/2016
 */

#ifndef SERIAL_IO_H_
#define SERIAL_IO_H_

#include "../../../Definitions.h"

#include <iosys.h>
#include <serial.h>
#include <pins.h>

#ifdef DEBUG_SERIAL_IO__
#include <stdio.h>
#endif

#define INDEX_OF_PC104_PORT 0
#define INDEX_OF_SPEC_REC 2

namespace Serial_IO {
	static int fdPC;

	//void startSerialReadTask();
	void serialReadTask(void * pd);

	int SerialRxFlush(int fd);

	inline void initSerial()
	{

	#ifdef DEBUG_SERIAL_IO__
		iprintf("Initializing Serial\n");
	#endif

		SerialClose(1);
		Pins[32].function(PIN_32_UART1_RXD);
		Pins[34].function(PIN_34_UART1_TXD);
		fdPC = OpenSerial(1, 115200, 1, 8, eParityNone);

	#ifdef DEBUG_SERIAL_IO__
		iprintf("Leaving Serial Init\n");
	#endif
	}

	inline int writePend(int* fileDesc, char* data, const int length)
	{
	#ifdef OLD_SERIAL_TRANSMIT
		for(int i = 0; i < 57; i++)
		{
			write(*fileDesc, &data[i], 1);
		}
	#else
		//writeall returns length if succesful
		//or negative if error,
		//TODO do something on error?
		return writeall(*fileDesc, data, length);
	#endif

	#ifdef DEBUG_SERIAL_IO__
		iprintf("Printing to Serial Port\n");
		for(int i = 0; i < 57; i++)
		{
			iprintf("%2x ", data[i]);
		}
	#endif
		//return 0;
	}

};

#endif /* SERIAL_IO_H_ */
