/*
 * UDPclass.cpp
 *
 *  Created on: March 2, 2015
 *      Author: Matt Gramlich
 *      Update: Modified for GENVII Autopilot, Scott Harper 8/2/2016
 */
#ifndef NBtoPC104_H
#define NBtoPC104_H

#include "../../../Definitions.h"

//externally linked union types are defined at the bottom

namespace NBtoPC104
{
/*
Do not use pointers to access this struct's data, pointers to data members
	do not necessarily inherit the ((packed)) attribute
Length of msg is 57 bytes
Do not use this struct else where, this struct is already initialized
	below in the union as msg.
Include this header and write to msg below, send data by using the
	array 'data'
This message goes to the I7 computer. The namespace member 'msg' is big endian, and needs
	to be changed to little endian before sending. Write data to 'msg', then call
	copyDataChangeEndian() and send the member 'littleEndianData'
*/



	struct __attribute__((packed)) msg_t
	{
		uint8_t H1;
		uint8_t H2;
		uint8_t H3;

		int32_t acc_x;
		int32_t acc_y;
		int32_t acc_z;
		int32_t rate_p;
		int32_t rate_q;
		int32_t rate_r;

		uint8_t control;

		uint16_t throttle;
		uint16_t aileron_right;
		uint16_t elevator_left;
		uint16_t aileron_left;
		uint16_t elevator_right;
		uint16_t rudder;
		uint16_t gear;
		uint16_t pControl;

		uint8_t counter;
	};

	const int size = sizeof(msg_t);

/*
	static union  {
			msg_t msg;
			BYTE data[size+3];
			//char dataChar[size+3];
		};

	static union  {
			//msg_t littleEndianMsg;
			BYTE littleEndianData[size+3];
			char littleEndianDataChar[size+3];
		};
*/
	 union bigEndianMsg_t {
		msg_t msg;
		BYTE data[size+3];
		char dataChar[size+3];
	};

	 union littleEndianData_t {
		//msg_t littleEndianMsg;
		BYTE littleEndianData[size+3];
		char littleEndianDataChar[size+3];
	};

	//static gives automatic external linkage, allowing this header to be
	//included in multiple source files without multiple Declaration errors
	/*inline void copyDataChangeEndian()
	{
		EndianSwap::changeToLittleEndian(littleEndianData, data, &size);
	}*/

	inline void copyDataChangeEndian(BYTE* LE, BYTE* BE, const int* length)
	{
		EndianSwap::changeToLittleEndian(LE, BE, length);
	}

extern NBtoPC104::bigEndianMsg_t NBtoPC104msg;
extern NBtoPC104::littleEndianData_t NBtoPC104msgData;

}
#endif
