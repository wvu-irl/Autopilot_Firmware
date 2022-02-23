/*
 * Comm.h
 *
 *  Created on: Apr 10, 2015
 *      Author: Matt Gramlich
 *      Update: Modified for GENVII Autopilot, Scott Harper 8/2/2016
 */

#ifndef COMM_H_
#define COMM_H_

#include "../../Definitions.h"

#include <ucos.h>
#include <startnet.h>
#include "EndianSwap.h"

extern OS_SEM NBtoPCSem;

//#include "../WatchDog/WatchDog.h"
#include "msgs/NBtoPCmsg.h"

//extern defines this msg for all recursive includes below
extern NBtoPC::bigEndianMsg_t NBtoPCmsg;
extern NBtoPC::littleEndianData_t NBtoPCmsgData;

#include "Serial/Serial_IO.h"

#ifdef DEBUG_COMM__
#define DEBUG_PRINT(args...) iprintf(args)
#else
//#define DEBUG_PRINT(args...)
#endif


namespace Comm
{

	//DWORD commStack[USER_TASK_STK_SIZE];

	void commWriteTask(void*);
	void commReadTask(void*);


	BYTE startCommTask();

#endif /* COMM_H_ */
