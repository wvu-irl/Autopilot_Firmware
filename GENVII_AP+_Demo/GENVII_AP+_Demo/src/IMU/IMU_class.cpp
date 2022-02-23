/*
 * IMU_class.cpp
 *
 *  Created on: Jan 23, 2015
 *  Author: Matt Gramlich
 *  	Derived from Quad_IMU.cpp
 *  			Author: Tanmay Mandal
 *  					Scott Harper
 *
 */

#include "IMU_class.h"
#include "../../Definitions.h"

#ifdef IMU_DEBUG
#include <stdio.h>
#endif
/*
IMU_class::IMU_class(int dspiChannelInput, int32_t* IMU_Data) {

	dataBuffer = IMU_Data;
	dspiChannel = dspiChannelInput;

	DSPIInit(dspiChannel,dspiBaudRate,transferSizeBits
			,CSPolarityIdle,1,1,1,0,0,0);//initializing SPI for IMU

}*/


IMU_class::IMU_class(int dspiChannelInput) {
//	timer1=HiResTimer::getHiResTimer(1);
//	timer1->init();
//

#ifdef MOD54415
					//order of array
					//Input, Output, Chip Select, Clock
	int SPIPins[12] = {25, 27, 28, 30, //SPI1
					   3 , 4 , 38, 29,  //SPI2
					   21, 22, 23, 24}; //SPI3
	J2[SPIPins[(dspiChannelInput-1)*4+ 0]].function(1);//SPI Input
	J2[SPIPins[(dspiChannelInput-1)*4+ 1]].function(1);//SPI Out
	J2[SPIPins[(dspiChannelInput-1)*4+ 2]].function(1);//SPI chip select 0
	J2[SPIPins[(dspiChannelInput-1)*4+ 3]].function(1);//SPI clock
#endif

#if (defined NANO54415)
	int SPIPins[12] = 	{33, 35, 15, 31, 	//SPI1
						24 , 26 , 28, 30,  	//SPI2
						32, 34, 36, 38}; 	//SPI3

	Pins[SPIPins[(dspiChannelInput-1)*4+ 0]].function(1);//SPI Input
	Pins[SPIPins[(dspiChannelInput-1)*4+ 1]].function(1);//SPI Out
	Pins[SPIPins[(dspiChannelInput-1)*4+ 2]].function(1);//SPI chip select 0
	Pins[SPIPins[(dspiChannelInput-1)*4+ 3]].function(1);//SPI clock

#endif

	dspiChannel = dspiChannelInput;
	for(int i = 0; i < 64; i++)
		IMU_DATA_BUFFER[i] = 0;

	OSSemInit(&SPISEM, 0);

#ifdef DEBUG_IMU__
	iprintf("initializing %i\n",
#endif

			DSPIInit(dspiChannel,dspiBaudRate,transferSizeBits,0x00
								,CSPolarityIdle,0x01,1,1,5,0)

#ifdef DEBUG_IMU__
			);//initializing SPI for IMU
#else
			;
#endif

#ifdef CHANGE_DECIMATION__
#ifdef DEBUG_IMU__
	iprintf("Writing Decimation Setting %i\n",
#endif

	DSPIStart(dspiChannel,IMU_decimation,NULL,10,&SPISEM,false,1)

#ifdef DEBUG_IMU__
			);
#else
	;
#endif

	OSSemPend(&SPISEM, 0);

#endif
}

void IMU_class::IMU_Read_Task(void* data) //no limit to this task, runs as much as it can, recommend low priority
{
	IMUTaskStruct* inputArgs = (IMUTaskStruct*)data; //cast input data to taskStruct defined in header
	bool runLoop = inputArgs->arrSize > 0; //eval size of array before for speed
	uint8_t i = 0; //declare iterator ahead for speed

	while(runLoop) // if there are elements, run the task, else end the task
	{
		for(i = 0; i < inputArgs->arrSize; i++)
			inputArgs->IMUArr[i].readAll();
	}
}

/*    	for(int i = 0; i < 36; i++)
    	    	{
    	    		iprintf("%3X\n",IMU1.IMU_DATA_BUFFER[i]);
    	    	}
    	printf("%3f\n", IMU1.dataBuffer[0]*.00025/65536.0);
    	printf("%3f\n", IMU1.dataBuffer[1]*.00025/65536.0);
    	printf("%3f\n", IMU1.dataBuffer[2]*.00025/65536.0);
    	printf("%3f\n", IMU1.dataBuffer[3]*.00025/65536.0);
    	printf("%3f\n", IMU1.dataBuffer[4]*.02/65536.0);
    	printf("%3f\n", IMU1.dataBuffer[5]*.02/65536.0);
    	printf("%3f\n", IMU1.dataBuffer[6]*.02/65536.0);
    	printf("%3f\n", IMU1.dataBuffer[7]*.00025/65536.0);
    	printf("%3f\n", IMU1.dataBuffer[8]*.00025/65536.0); */


