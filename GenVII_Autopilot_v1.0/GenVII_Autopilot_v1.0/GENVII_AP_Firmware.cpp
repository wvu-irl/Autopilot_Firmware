/*
 * Generation VII Autopilot Firmware v1.0
 * For use with GenVII Avionics: Autopilot v 1.1
 * Author: Scott Harper,sharper9@mix.wvu.edu
 * Date: 8/09/2016
 *
 */

//Libraries to be included
///included
#include "Definitions.h"

#include "predef.h"
#include <stdio.h>
#include <startnet.h>
#include <autoupdate.h>
#include <smarttrap.h>
#include <taskmon.h>
#include <pins.h>
#include <pitr_sem.h>
#include <basictypes.h>
#include <ucos.h>
#include <constants.h>
#include <cfinter.h>
#include <NetworkDebug.h>
#include <HiResTimer.h>
#include <serial.h>

//#include "src/misc/Accessory_Functions_GenVII.h"
#include "src/misc/PINS_Definations.h"

#include "src/IMU/IMU_class.h"
#include "src/comm/Comm.h"
#include "src/config_storage/Config.h"
#include "src/PWM/Surfaces.h"

//externally linked semaphores
OS_SEM PITWriteSem;
OS_SEM PC104toNBSem;
OS_SEM NBtoPC104Sem;
OS_SEM PITCSurfSem;

//externally linked msg definitions
NBtoPC104::bigEndianMsg_t NBtoPC104msg;
PC104toNB::bigEndianMsg_t PC104toNBmsg;

HiResTimer* timer = HiResTimer::getHiResTimer(1);

extern "C" {
void UserMain(void * pd);
}

const char * AppName = APPNAME
;

void UserMain(void* pd) {
	/////Usual Routine

	InitializeStack();
	OSChangePrio(USER_MAIN_PRIO);
	EnableAutoUpdate();
	//EnableTaskMonitor();
	//EnableSmartTraps();

	if (!Config::checkConfig()) {
		iprintf("Config MisMatch! Writing config and rebooting\n");
		Config::updateConfig();
	}

	//initPINS();

	timer->clearInterruptFunction();
	timer->init();
	timer->start();

	OSSemInit(&NBtoPC104Sem, 1); //needs to initialize to 1, so something can take it, then post it
	OSSemInit(&PC104toNBSem, 1);
	OSSemInit(&PITCSurfSem, 1);

	OSSemInit(&PITWriteSem, 0);
	InitPitOSSem(2, &PITWriteSem, PIT_SEM_RATE);
	//InitPitOSSem(0, &PITCSurfSem, PIT_SEM_RATE);

	IMU_class::IMU_class IMU(3); //creates new IMU object on bus

	//GPIO Definitions
	//LEDS
	J2[45].function(0);
	J2[45] = 1;
	J1[6].function(0);
	J1[6] = 1;
	J1[5].function(0);
	J1[5] = 1;

	//For PWM
	//AND and Multiplexer pins
	J2[26].function(0); //CH1 Enable
	J2[32].function(0); //CH2 Enable
	J2[33].function(0); //CH3 Enable
	J2[34].function(0); //CH4 Enable
	J2[36].function(0); //CH5 Enable
	J2[37].function(0); //CH6 Enable

	//Autopilot Activate
	J2[15].function(0);
	J2[15] = 1;

	//Actuator Control Channels
	J2[26] = 1; //Throttle
	J2[32] = 1; //Right Aileron
	J2[33] = 1; //Right Elevator
	J2[34] = 1; //Rudder
	J2[36] = 1; //Left Elevator
	J2[37] = 1; //Left Aileron

	//Buzzer
	J2[38].function(0);
	J2[38] = 0; //MAKE IT STOP

	iprintf("\n%s Application started\n", AppName);

	Comm::startCommTask();
	Surfaces::startPwmControlTask();

	while (1) {
		IMU.readAll();

		OSSemPend(&IMU.SPISEM, 0);

		if (OSSemPend(&NBtoPC104Sem, 10) == OS_NO_ERR) //blocks until MSG semaphore is released and grabs it. if Half a second passes ignores and moves on
		{
			//Write IMU to Data Packet
			NBtoPC104msg.msg.acc_x = IMU.dataBuffer[1];
			NBtoPC104msg.msg.acc_y = IMU.dataBuffer[2];
			NBtoPC104msg.msg.acc_z = IMU.dataBuffer[3];
			NBtoPC104msg.msg.rate_p = IMU.dataBuffer[4];
			NBtoPC104msg.msg.rate_q = IMU.dataBuffer[5];
			NBtoPC104msg.msg.rate_r = IMU.dataBuffer[6];

			//DEBUG_PRINT("\nIMU DATA:\nA:X%8i\nA:Y%8i\nA:Z%8i\nR:P%8i\nR:Q%8i\nR:R%8i\n",
			//NBtoPC104msg.msg.acc_x,NBtoPC104msg.msg.acc_y,NBtoPC104msg.msg.acc_z,
			//NBtoPC104msg.msg.rate_p,NBtoPC104msg.msg.rate_q,NBtoPC104msg.msg.rate_r);

			OSSemPost(&NBtoPC104Sem); //releases grabbed semaphore
		}

	} //While
} //Main Task

