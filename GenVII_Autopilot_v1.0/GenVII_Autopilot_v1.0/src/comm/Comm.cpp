/*
 * Comm.cpp
 *
 *  Created on: Apr 10, 2015
 *      Author: Matt Gramlich
 *      Update: Modified for GENVII Autopilot, Scott Harper 8/2/2016
 */

#include "Comm.h"
#include "Serial/Serial_IO.h"
#include "msgs/PC104toNBmsg.h"
#include "msgs/NBtoPC104msg.h"

extern OS_SEM PITWriteSem;
extern OS_SEM PITCSurfSem;

//See Serial_IO.h, initSerial for indexes
#define SERIAL_INDEX_UART0 serialFd[1]
#define SERIAL_INDEX_UART2 serialFd[0]

#define LED_FLASH

void Comm::commWriteTask(void* inputArg) {
	NBtoPC104msg.msg.H1 = 'S';
	NBtoPC104msg.msg.H2 = 'H';
	NBtoPC104msg.msg.H3 = '1';

	NBtoPC104msg.msg.counter = 0;

	while (1) {
		DEBUG_PRINT_NET("\r\nIn Comm Task\nPITSEM Block\r\n");

		OSSemPend(&PITWriteSem, 100);

		//TODO should the timeout be changed to allow flow on error?
		//or is communication fatal enough to not be recoverable?
		//DEBUG_PRINT_NET("Comm Write Task:: Attempting to grab NBtoPC104 SEM\r\n");

		OSSemPend(&NBtoPC104Sem, 100);

		NBtoPC104msg.msg.counter++;
		OSSemPost(&NBtoPC104Sem);

		Serial_IO::writePend(&Serial_IO::serialFd[INDEX_OF_PC104_PORT],
				NBtoPC104msg.dataChar, NBtoPC104::size);

		/*
		 PC104toNBmsg.msg.throttle = NBtoPC104msg.msg.throttle;
		 PC104toNBmsg.msg.aileron_right = NBtoPC104msg.msg.aileron_right;
		 PC104toNBmsg.msg.aileron_left = NBtoPC104msg.msg.aileron_left;
		 PC104toNBmsg.msg.elevator_left = NBtoPC104msg.msg.elevator_left;
		 PC104toNBmsg.msg.rudder = NBtoPC104msg.msg.rudder;
		 PC104toNBmsg.msg.elevator_right = NBtoPC104msg.msg.elevator_right;
		 */

		OSSemPost(&PITCSurfSem);

#ifdef LED_FLASH
		J2[45] = J2[45] ^ 1;
#endif

		//DEBUG_PRINT("\nComm Write Task:: Finished\n");

	}
}

void Comm::commReadTask(void* inputArg) {

}

void Comm::specReadTask(void* inputArg) {
	char specData[32] = { 0 };

	//Initial Locations
	PC104toNBmsg.msg.throttle = 300;
	PC104toNBmsg.msg.aileron_right = 1025;
	PC104toNBmsg.msg.aileron_left = 1025;
	PC104toNBmsg.msg.elevator_left = 1025;
	PC104toNBmsg.msg.rudder = 1025;
	PC104toNBmsg.msg.elevator_right = 1025;

	while (1) {
		OSSemPend(&PC104toNBSem, 100);

		read(Serial_IO::serialFd[INDEX_OF_SPEC_REC], &specData[0], 1);
		if ((unsigned char) specData[0] == 0xFF) {
			read(Serial_IO::serialFd[INDEX_OF_SPEC_REC], &specData[1], 1);
			if ((unsigned char) specData[1] == 0xFF) {
				for (int i = 2; i < 5; i++)
					read(Serial_IO::serialFd[INDEX_OF_SPEC_REC], &specData[i],
							1);
				if ((unsigned char) (specData[4] & 0x80) == 0x80)
					for (uint8_t i = 5; i < sizeof(specData); i++) {
						read(Serial_IO::serialFd[INDEX_OF_SPEC_REC],
								&specData[i], 1);
					}
			}
		}

		//for(uint8_t i = 0; i < sizeof(specData);i++)
		//	DEBUG_PRINT("%4X ",(uint8_t)specData[i]);
		//DEBUG_PRINT("\n");

		NBtoPC104msg.msg.throttle = (uint16_t) ((specData[4] & 0x07) * 256) //correct
		+ (uint8_t) (specData[5]);

		NBtoPC104msg.msg.aileron_right =
				(uint16_t) ((specData[20] & 0x07) * 256) //correct
				+ (uint8_t) (specData[21]);

		NBtoPC104msg.msg.aileron_left = (uint16_t) ((specData[6] & 0x07) * 256) //correct
		+ (uint8_t) (specData[7]);

		NBtoPC104msg.msg.elevator_left =
				(uint16_t) ((specData[24] & 0x07) * 256) //correct
				+ (uint8_t) (specData[25]);

		NBtoPC104msg.msg.gear = (uint16_t) ((specData[30] & 0x07) * 256)
				+ (uint8_t) (specData[31]);

		NBtoPC104msg.msg.rudder = (uint16_t) ((specData[8] & 0x07) * 256)
				+ (uint8_t) (specData[9]);

		NBtoPC104msg.msg.elevator_right = (uint16_t) ((specData[10] & 0x07)
				* 256) + (uint8_t) (specData[11]);

		NBtoPC104msg.msg.control = (uint16_t) ((specData[26] & 0x07) * 256)
				+ (uint8_t) (specData[27]);

		OSSemPost(&PC104toNBSem);

#ifdef LED_FLASH
		J1[6] = J1[6] ^ 1;
#endif

		/*

		 DEBUG_PRINT("T:%u ", NBtoPC104msg.msg.throttle);
		 DEBUG_PRINT("LA:%u ", NBtoPC104msg.msg.aileron_left);
		 DEBUG_PRINT("RA:%u ", NBtoPC104msg.msg.aileron_right);
		 DEBUG_PRINT("LE:%u ", NBtoPC104msg.msg.elevator_left);
		 DEBUG_PRINT("RE:%u ", NBtoPC104msg.msg.elevator_right);
		 DEBUG_PRINT("R:%u ", NBtoPC104msg.msg.rudder);
		 DEBUG_PRINT("G:%u  ", NBtoPC104msg.msg.gear);
		 DEBUG_PRINT("C:%u", NBtoPC104msg.msg.control);
		 DEBUG_PRINT("\n");
		 */

	}
}

BYTE Comm::startCommTask() {
	Serial_IO::initSerial();

	DEBUG_PRINT("Starting Comm Task\n");

	OSSimpleTaskCreatewName(Comm::commWriteTask, COMM_WRITE_TASK_PRIO,
			"Comm Write Task");
	//OSSimpleTaskCreatewName(Comm::commReadTask, COMM_READ_TASK_PRIO, "Comm Read Task");
	OSSimpleTaskCreatewName(Comm::specReadTask, SPEC_READ_TASK_PRIO,
			"Spec Read Task");
	//UDP_IO::startUDPReadTask(I7toNB1msgData.data);

	DEBUG_PRINT("After Comm Task\n");

	DEBUG_PRINT("Leaving Comm Task\n");

	return 0;
}

