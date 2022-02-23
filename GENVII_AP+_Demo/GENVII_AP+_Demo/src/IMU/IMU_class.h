/*
 * IMU_class.h
 *
 *  Created on: Jan 23, 2015
 *  Author: Matt Gramlich
 *  	Derived from Quad_IMU.cpp
 *  			Author: Tanmay Mandal
 *  					Scott Harper
 *
 */
#ifndef IMU_CLASS_H
#define IMU_CLASS_H

//#include "../../Definitions.h"

#define CHANGE_DECIMATION__

#include <dspi.h>
#include <pins.h>

#define xghigh 0x12  // 00010010
#define xglow  0x10  // 00010000
#define yghigh 0x16  // 00010110
#define yglow  0x14  // 00010100
#define zghigh 0x1A  // 00011010
#define	zglow  0x18  // 00011000
#define	xahigh 0x1E  // 00011110
#define xalow  0x1C  // 00011100
#define	yahigh 0x22  // 00100010
#define	yalow  0x20  // 00100000
#define	zahigh 0x26  // 00100110
#define	zalow  0x24  // 00100100
#define xdahigh 0x42
#define xdalow  0x40
#define ydahigh 0x46
#define ydalow  0x44
#define zdahigh 0x4A
#define zdalow  0x48
#define xdvhigh 0x4E
#define xdvlow  0x4C
#define ydvhigh 0x52
#define ydvlow  0x50
#define zdvhigh 0x56
#define zdvlow  0x54


//decimation setup
//decimation rate set to 20 Point 0x13, refresh rate is ~124Hz
//decimation rate 42 Point 0x2A, refresh rate is ~55Hz
//decimation rate 45 Point 0x2D, refresh rate is ~52Hz
#define changePageTo3 0x80,0x03 //10000000 00000011 write register 0x00 to 0x03
#define decRegLow 	  0x8C,0x2A //10001100 00010011 write register 0x0C to
#define decRegHigh    0x8D,0x00 //10001101 00000000 write register 0x0D to 0x00
#define changePageTo0 0x80,0x00 //10000000 00000000 write register 0x00 to 0x00
#define zeros		  0x00,0x00

#define dspiBaudRate 10000000
#define transferSizeBits 16
#define CSPolarityIdle 0x0F //Chip select polarity at idle

#define COMMAND_LENGTH 50
static BYTE __attribute__ ((aligned(2)))
								IMU_command_all[COMMAND_LENGTH] = {xahigh, 0, xalow, 0, yahigh, 0, yalow, 0, zahigh, 0, zalow, 0,
																   xghigh, 0, xglow, 0, yghigh, 0, yglow, 0, zghigh, 0, zglow, 0,
																   xdahigh,0, xdalow,0, ydahigh,0, ydalow,0, zdahigh,0, zdalow,0,
																   xdvhigh,0, xdvlow,0, ydvhigh,0, ydvlow,0, zdvhigh,0, zdvlow,0,
																	0,0};

static BYTE __attribute__ ((aligned(2))) IMU_decimation[10] = {changePageTo3, decRegLow, decRegHigh, changePageTo0, zeros};

class IMU_class {
public:
	BYTE dspiChannel;
	OS_SEM SPISEM;
	int numAxis;

	union __attribute__ ((__packed__))
	{
		volatile int32_t __attribute__ ((aligned(2))) dataBuffer[16];
		volatile BYTE __attribute__ ((aligned(2))) IMU_DATA_BUFFER[64];
	};

	struct IMUTaskStruct
	{
		IMU_class* IMUArr;
		uint8_t arrSize;
	};

	//IMU_class(int dspiChannelInput, int32_t* inputDataBuffer);
	IMU_class(int dspiChannelInput);

	//input data should be structure
	void IMU_Read_Task(void* data);

	inline int readAll()
	{
		return DSPIStart(dspiChannel,IMU_command_all,IMU_DATA_BUFFER,COMMAND_LENGTH,&SPISEM,false,1);
	};

	//returns true if two or more axis of the IMU have data that does not equal zero
	inline bool checkData()
	{
		numAxis = 0;
		for(int i = 0; (numAxis < 3) && i < 6; i++)
		{
			//DEBUG_PRINT("\ndatabuffer %i\n",dataBuffer[i]);
			if(dataBuffer[i] != -1)
				numAxis++;
		}
		return !(numAxis < 3);
	}


private:

};

#endif
