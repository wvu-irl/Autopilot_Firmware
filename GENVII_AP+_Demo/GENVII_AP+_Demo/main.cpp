#include <predef.h>
#include <stdio.h>
#include <ctype.h>
#include <startnet.h>
#include <autoupdate.h>
#include <dhcpclient.h>
#include <smarttrap.h>
#include <taskmon.h>
#include <NetworkDebug.h>
#include <pins.h>
#include <SimpleAD.h>
#include <multican.h>
#include <string.h>

#include "Definitions.h"
#include "src/PWM/PWM.h"
#include "src/comm/Comm.h"
#include "src/IMU/IMU_class.h"

OS_SEM PITWriteSem;
OS_SEM NBtoPCSem;

NBtoPC::bigEndianMsg_t NBtoPC104msg;

extern unsigned long CPU_CLOCK;

extern "C" {
void UserMain(void * pd);
}

void UserMain(void * pd) {
	//EnableOSStackProtector(); // TODO: Uncomment #define UCOS_STACKOVERFLOW in predef.h, rebuild libraries
	InitializeStack();
	GetDHCPAddressIfNecessary();
	OSChangePrio(MAIN_PRIO);
	EnableAutoUpdate();
	StartHTTP();
	EnableTaskMonitor();

#ifndef _DEBUG
	EnableSmartTraps();
#endif

#ifdef _DEBUG
	InitializeNetworkGDB_and_Wait();
#endif

// GPIO Setup
	Pins[19].function(PIN_13_GPIO); //LED
	Pins[19] = 1;

	Pins[23].function(PIN_23_GPIO); //Autopilot PWM Control
	Pins[23] = 1; //Control

//Input Specific Initialization
	IMU_class IMU(3); //Creates and IMU Object on SPI3 (On-Board IMU)

#ifdef Serial_Input
	OSSimpleTaskCreatewName(Comm::commReadTask, COMM_READ_TASK_PRIO, "Comm Read Task");
#endif

#ifdef ADC_Input
	InitSingleEndAD();
	uint32_t pot;
#endif

//Output Specific Initialization
#ifdef PWM_Output
	PWM Servo(0xA0, 50);
#endif

#ifdef CAN_Output
#define CAN_MODULE (0)
#define BAUD_RATE 1000000

	if (CAN_MODULE == 0) {
		Pins[29].function(PIN_29_CAN0_RX);
		Pins[27].function(PIN_27_CAN0_TX);
	} else {
		Pins[20].function(PIN_20_CAN1_RX);
		Pins[22].function(PIN_22_CAN1_TX);
	}

	MultiCanInit(CAN_MODULE, BAUD_RATE, 0, 4);
	OS_FIFO fifo;
	OSFifoInit(&fifo);
	int chan1 = RegisterMultiCanRxFifo(CAN_MODULE, NormToNbId( 0x123 ), &fifo);

#endif

	int16_t input;
	char buffer[2];

	iprintf("Application started\n");
	while (1) {

#ifdef ADC_Input
		StartAD();
		while (!ADDone())
			asm("nop");
		pot = (double) GetADResult(1);

		input = (double) pot * 2000 / 32768; //(0-2000)
#endif

#ifdef PWM_Output
		Servo.setPWM(input);
#endif

#ifdef CAN_Output

		// read CAN buffer
		// this must be done before every send to clear the fifo
		CanRxMessage can_msg(&fifo, 1); //Wait up to 1 tick

		// break
		buffer[0] = (uint8_t) ((input >> 8) & 0xff);
		buffer[1] = (uint8_t) (input & 0xff);

		MultiCanSendMessage(CAN_MODULE, NormToNbId( 1 ), (PBYTE) buffer,
				strlen(buffer), 50);

#endif

		IMU.readAll();
		OSSemPend(&IMU.SPISEM, 0);

		if (OSSemPend(&NBtoPCSem, TICKS_PER_SECOND/10) == OS_NO_ERR) //blocks until MSG semaphore is released and grabs it. if 1/10 of a second passes ignores and moves on
		{
			NBtoPCmsg.msg.acc_x = IMU.dataBuffer[1];
			NBtoPCmsg.msg.acc_y = IMU.dataBuffer[2];
			NBtoPCmsg.msg.acc_z = IMU.dataBuffer[3];
			NBtoPCmsg.msg.rate_p = IMU.dataBuffer[4];
			NBtoPCmsg.msg.rate_q = IMU.dataBuffer[5];
			NBtoPCmsg.msg.rate_r = IMU.dataBuffer[6];

			OSSemPost(&NBtoPCSem); //releases grabbed semaphore
		}

		DEBUG_PRINT("%i\n", input);

		Pins[19] = Pins[19] ^ 1;
		OSTimeDly(TICKS_PER_SECOND / 20);
	}
}
