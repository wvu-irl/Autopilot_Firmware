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
					//order of array
					//Input, Output, Chip Select, Clock
	int SPIPins[12] = {25, 27, 28, 30, //SPI1
					   3 , 4 , 38, 29,  //SPI2
					   21, 22, 23, 24}; //SPI3
	J2[SPIPins[(dspiChannelInput-1)*4+ 0]].function(1);//SPI Input
	J2[SPIPins[(dspiChannelInput-1)*4+ 1]].function(1);//SPI Out
	J2[SPIPins[(dspiChannelInput-1)*4+ 2]].function(1);//SPI chip select 0
	J2[SPIPins[(dspiChannelInput-1)*4+ 3]].function(1);//SPI clock

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

/*
inline void IMU_class::readAll() {

	DSPIStart(dspiChannel,IMU_command_all,IMU_DATA_BUFFER,28,&test);
}
	//Now Defunct code, no need for bitwise ops with unions.

	//dataBuffer[1]=(short int)(IMU_DATA_BUFFER[2]*256+IMU_DATA_BUFFER[3]);//ZERO
	dataBuffer[0]= ((int32_t) IMU_DATA_BUFFER[4] << 24
			| (int32_t) IMU_DATA_BUFFER[5] << 16 | (int32_t) IMU_DATA_BUFFER[6] << 8
			| (int32_t) IMU_DATA_BUFFER[7]); //X ACCEL
	dataBuffer[1]= ((int32_t) IMU_DATA_BUFFER[8] << 24
			| (int32_t) IMU_DATA_BUFFER[9] << 16 | (int32_t) IMU_DATA_BUFFER[10] << 8
			| (int32_t) IMU_DATA_BUFFER[11]); //Y ACCEL
	dataBuffer[2]= ((int32_t) IMU_DATA_BUFFER[12] << 24
			| (int32_t) IMU_DATA_BUFFER[13] << 16 | (int32_t) IMU_DATA_BUFFER[14] << 8
			| (int32_t) IMU_DATA_BUFFER[15]); //Z ACCEL
	dataBuffer[3]= ((int32_t) IMU_DATA_BUFFER[16] << 24
			| (int32_t) IMU_DATA_BUFFER[17] << 16 | (int32_t) IMU_DATA_BUFFER[18] << 8
			| (int32_t) IMU_DATA_BUFFER[19]); //P RATE
	dataBuffer[4]= ((int32_t) IMU_DATA_BUFFER[20] << 24
			| (int32_t) IMU_DATA_BUFFER[21] << 16 | (int32_t) IMU_DATA_BUFFER[22] << 8
			| (int32_t) IMU_DATA_BUFFER[23]); //Q RATE
	dataBuffer[5]= ((int32_t) IMU_DATA_BUFFER[24] << 24
			| (int32_t) IMU_DATA_BUFFER[25] << 16 | (int32_t) IMU_DATA_BUFFER[26] << 8
			| (int32_t) IMU_DATA_BUFFER[27]); //R RATE
*/
	//iprintf("IMU1: %6d %6d %6d %6d %6d %6d\n",dataBuffer[0],dataBuffer[1],dataBuffer[2],dataBuffer[3],dataBuffer[4],dataBuffer[5]);



