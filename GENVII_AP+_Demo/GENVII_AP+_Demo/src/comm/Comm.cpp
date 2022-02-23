/*
 * Comm.cpp
 *
 *  Created on: Apr 10, 2015
 *      Author: Matt Gramlich
 *      Update: Modified for GENVII Autopilot, Scott Harper 8/2/2016
 */

#include "Comm.h"
#include "Serial/Serial_IO.h"
#include "msgs/NBtoPCmsg.h"

extern OS_SEM PITWriteSem;
extern OS_SEM PITCSurfSem;

#define LED_FLASH

void commWriteTask(void* inputArg) {
	NBtoPCmsg.msg.H1 = 'A';
	NBtoPCmsg.msg.H2 = 'P';
	NBtoPCmsg.msg.H3 = '+';

	NBtoPCmsg.msg.counter = 0;

	while (1) {
		DEBUG_PRINT_NET("\r\nIn Comm Task\nPITSEM Block\r\n");

		OSSemPend(&PITWriteSem, 100);

		//TODO should the timeout be changed to allow flow on error?
		//or is communication fatal enough to not be recoverable?
		//DEBUG_PRINT_NET("Comm Write Task:: Attempting to grab NBtoPC SEM\r\n");

		OSSemPend(&NBtoPCSem, 100);

		NBtoPCmsg.msg.counter++;
		OSSemPost(&NBtoPCSem);

		Serial_IO::writePend(&Serial_IO::fdPC,
				NBtoPCmsg.dataChar, NBtoPC::size);

		//DEBUG_PRINT("\nComm Write Task:: Finished\n");

	}
}

void commReadTask(void* inputArg) {

}

BYTE startCommTask() {
	Serial_IO::initSerial();

	DEBUG_PRINT("Starting Comm Task\n");

	OSSimpleTaskCreatewName(Comm::commWriteTask, COMM_WRITE_TASK_PRIO,
			"Comm Write Task");
	//OSSimpleTaskCreatewName(Comm::commReadTask, COMM_READ_TASK_PRIO, "Comm Read Task");

	//UDP_IO::startUDPReadTask(I7toNB1msgData.data);

	DEBUG_PRINT("After Comm Task\n");

	DEBUG_PRINT("Leaving Comm Task\n");

	return 0;
}

