/*
 * CAN.h
 *
 *  Created on: Nov 17, 2017
 *      Author: Scott(IRL)
 */

#ifndef CAN_H_
#define CAN_H_

#include <iosys.h>
#include <ucos.h>
#include <multican.h>
#include <pins.h>

namespace CAN_IO {

#define CAN_MODULE (0)
//#define CAN_MODULE (1)

#define BAUD_RATE 500000

void ConfigureCanPins( uint32_t canModuleNumber);

void initCAN();

}
#endif /* CAN_H_ */
