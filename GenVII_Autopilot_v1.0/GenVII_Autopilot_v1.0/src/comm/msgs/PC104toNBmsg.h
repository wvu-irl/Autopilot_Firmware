/*
 * UDPclass.cpp
 *
 *  Created on: March 2, 2015
 *      Author: Matt Gramlich
 *      Update: Modified for GENVII Autopilot, Scott Harper 8/2/2016
 */
#ifndef PC104toNB_H
#define PC104toNB_H

#include "../EndianSwap.h"

namespace PC104toNB
{
/*
Do not use pointers to access this struct's data, pointers to data members
	do not necessarily inherit the ((packed)) attribute
Length of msg is 13 bytes
Include this header and write to msg below, send data by using the
	array 'data'
This message comes from the I7 computer and the data is little endian, write UDP buffer
	to msg, then call copyDataChangeEndian() method and the result data is in bigEndianMsg
*/
	const static int size = 28;

	/*
	 * To do the endian swap, place the data in the msg backward, then swap bytes
	 */
	struct __attribute__((packed)) msg_t
	{

		uint16_t rudder;
		uint16_t elevator_right;
		uint16_t aileron_left;
		uint16_t elevator_left;
		uint16_t aileron_right;
		uint16_t throttle;

		uint8_t H3;
		uint8_t H2;
		uint8_t H1;
	};

	union littleEndianData_t
	{
		BYTE data[size+3];		//plus a few to be safe
		char dataChar[size+3];
	};

	union bigEndianMsg_t
	{
		msg_t msg;
		BYTE bigEndianData[size+3];
		char bigEndianDataChar[size+3];
	};

	inline void copyDataChangeEndian(BYTE* LE, BYTE* BE, const int* length)
	{
		EndianSwap::changeToBigEndian(LE, BE, length);
	}

	inline BYTE doChecksum(BYTE* LE, const int* length)
	{
		BYTE checksum = 0;
		for (uint8_t i = 3; i < *length - 1; i++)
		{
			checksum = (checksum + static_cast<uint8_t>(LE[i])) & 255;
		}
		return checksum;
	}

};

extern PC104toNB::bigEndianMsg_t PC104toNBmsg;
extern PC104toNB::littleEndianData_t PC104toNBmsgData;

#endif
