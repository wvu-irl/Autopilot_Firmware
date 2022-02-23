/*
 * surfaces.h
 *
 *  Created on: Aug 5, 2016
 *      Author: Scott Harper
 */

#ifndef SURFACES_H_
#define SURFACES_H_


#include "../../Definitions.h"
#include "PWM.h"

#include "../comm/msgs/PC104toNBmsg.h"
#include "../comm/msgs/NBtoPC104msg.h"

#ifdef DEBUG_SURFACES__
#define DEBUG_PRINT(args...) iprintf(args)
#else
//#define DEBUG_PRINT(args...)
#endif


namespace Surfaces
{

	//DWORD commStack[USER_TASK_STK_SIZE];

	void pwmControlTask(void*);

	BYTE startPwmControlTask();

}

#endif /* COMM_H_ */
