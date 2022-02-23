/*
 * surfaces.cpp
 *
 *  Created on: Aug 5, 2016
 *      Author: 115
 */

#include "Surfaces.h"
#include "PWM.h"
#include "../comm/msgs/PC104toNBmsg.h"
#include "../comm/msgs/NBtoPC104msg.h"

extern OS_SEM PITCSurfSem;

#define LED_FLASH

void Surfaces::pwmControlTask(void* inputArg)
{

	PWM::PWM throttle(0xA0, 200);
	PWM::PWM aileronRight(0xB0, 200);
	PWM::PWM elevatorRight(0xB2, 200);
	PWM::PWM rudder(0xA1, 200);
	PWM::PWM elevatorLeft(0xA2, 200);
	PWM::setPWMChannel aileronLeft(0xB1, 200);





	while(1)
	{
		//DEBUG_PRINT("\r\nIn Control Surfaces Task\nPITSEM Block\r\n");

		OSSemPend(&PITCSurfSem, 200);

		//TODO should the timeout be changed to allow flow on error?
		//or is communication fatal enough to not be recoverable?
		//DEBUG_PRINT_NET("Comm Write Task:: Attempting to grab NBtoPC104 SEM\r\n");

		throttle.setPWM(PC104toNBmsg.msg.throttle);
		aileronLeft.setPWM(PC104toNBmsg.msg.aileron_left);
		aileronRight.setPWM(PC104toNBmsg.msg.aileron_right);
		elevatorLeft.setPWM(PC104toNBmsg.msg.elevator_left);
		elevatorRight.setPWM(PC104toNBmsg.msg.elevator_right);
		rudder.setPWM(PC104toNBmsg.msg.rudder);

#ifdef LED_FLASH
			J1[5] = J1[5] ^ 1;
#endif

	}
}


BYTE Surfaces::startPwmControlTask()
{

		DEBUG_PRINT("Starting Control Surfaces Task\n");

		OSSimpleTaskCreatewName(Surfaces::pwmControlTask, CONTROL_PWM_TASK_PRIO, "Control Surfaces Task");

		DEBUG_PRINT("Control Surfaces Task Started\n");

	    return 0;
}






