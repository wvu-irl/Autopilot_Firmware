/*
 * Serial_IO.cpp
 *
 *  Created on: Mar 26, 2015
 *      Author: Matt Gramlich
 *      Update: Modified for GENVII Autopilot, Scott Harper 8/2/2016
 */

#include "Serial_IO.h"

void serialReadTask(void * pd)
{

};

int SerialRxFlush(int fd)
{
	//static char buf[2048];
	if (dataavail(fd))
		return read(fd, NULL, 2048);
	return 0;
};
