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

extern OS_SEM PC104toNBSem;
extern OS_SEM NBtoPC104Sem;

//#include "../WatchDog/WatchDog.h"
#include "msgs/PC104toNBmsg.h"
#include "msgs/NBtoPC104msg.h"

//extern defines this msg for all recursive includes below
extern NBtoPC104::bigEndianMsg_t NBtoPC104msg;
extern NBtoPC104::littleEndianData_t NBtoPC104msgData;

extern PC104toNB::bigEndianMsg_t PC104toNBmsg;
extern PC104toNB::littleEndianData_t PC104toNBmsgData;

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
	void specReadTask(void*);

	BYTE startCommTask(); //UDP Write and Serial Write are in the same Task
							   //UDP  Read and Serial  Read are in different Tasks
									//UDP Read is higher Priority than Serial Read


/**
 * returns true if messages read from serial and UDP are the same
 */
	inline bool compareMsgs(PC104toNB::bigEndianMsg_t UDPMsg, PC104toNB::bigEndianMsg_t serialMsg)
	{
		bool res = true;
		for(int i = 0; res && i < PC104toNB::size; i++)
		{
			res = UDPMsg.bigEndianData[i] == serialMsg.bigEndianData[i];
		}
		return res;
	}

}

#endif /* COMM_H_ */
