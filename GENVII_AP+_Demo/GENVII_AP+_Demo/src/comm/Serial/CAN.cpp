/*
 * CAN.cpp
 *
 *  Created on: Nov 17, 2017
 *      Author: Scott(IRL)
 */

#include "CAN.h"

void ConfigureCanPins(uint32_t canModuleNumber) {

#ifdef NANO54415_PIN_CONSTANT
	if (canModuleNumber == 0) {
		Pins[29].function(PIN_29_CAN0_RX);
		Pins[27].function(PIN_27_CAN0_TX);
	} else if (canModuleNumber == 1) {
		Pins[20].function(PIN_20_CAN1_RX);
		Pins[22].function(PIN_22_CAN1_TX);
	} else {
		iprintf("Invalid CAN module number\r\n");
	}
#endif
#ifdef MOD5441x_PIN_CONSTANT
	if ( canModuleNumber == 0 )
	{
		J2[42].function(PINJ2_42_CAN0_TX);
		J2[39].function(PINJ2_39_CAN0_RX );
	}
	else if (canModuleNumber == 1)
	{
		J2[44].function(PINJ2_44_CAN1_TX);
		J2[41].function(PINJ2_41_CAN1_RX );
	}
	else
	{
		iprintf("Invalid CAN module number\r\n");
	}
#endif
}
void initCAN() {

	MultiCanInit(CAN_MODULE, BAUD_RATE, 0, 4);
    OS_FIFO fifo;
    OSFifoInit(&fifo);
    int chan1 = RegisterMultiCanRxFifo(CAN_MODULE, NormToNbId( 0x123 ), &fifo);
    int chan2 = RegisterMultiCanRxFifo(CAN_MODULE, ExtToNbId( 0x1234 ), &fifo);

}
