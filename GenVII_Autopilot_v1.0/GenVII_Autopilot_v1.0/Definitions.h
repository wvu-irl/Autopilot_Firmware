/*
 * Definitions.h
 *
 *  Created on: May 26, 2015
 *      Author: Matt Gramlich
 *      Update: Modified for GENVII Autopilot, Scott Harper 8/2/2016
 */

#ifndef DEFINITIONS_H_
#define DEFINITIONS_H_

//PLATFORM DEFINITION
//A#define RED 1
#define GREEN 2

//#define IMU_DELTA_VALUES
#define SERIAL_OUTPUT
//#define ADC_VALUES

//#define I7IP "10.10.10.1"
#define PIT_SEM_RATE 50 //PIT Semaphore posting rate, in Hz, affects UDP and Serial msgs transmit speed
#define bootDelay 1
#define subnetMask "255.255.255.0"

#ifdef RED
#define localIP "10.10.10.2"
#define monitorPort 0
#endif
#ifdef GREEN
#define localIP "10.10.10.3"
#define monitorPort 0
#endif

//TASK PRIORITIES
#define COMM_WRITE_TASK_PRIO 50
#define COMM_READ_TASK_PRIO	52
#define SPEC_READ_TASK_PRIO	51
#define CONTROL_PWM_TASK_PRIO 53
#define USER_MAIN_PRIO 60

//DEBUG DEFINITIONS
//#define DEBUG_PRINT__
//#define DEBUG_COMM__
//#define DEBUG_SERIAL_IO__
//#define DEBUG_UDP_IO__
//#define DEBUG_IMU__

//Uncomment to setup network debugging
//#include <syslog.h>

#ifdef DEBUG_PRINT__
#define DEBUG_PRINT(args...) iprintf(args)
#undef PIT_SEM_RATE
#define PIT_SEM_RATE 20 //PIT Semaphore posting rate, in Posts/sec, aka Hz. Needs to be slower if debug printing
#else
#define DEBUG_PRINT(args...)
#endif

#ifdef SYSLOG_H
#define DEBUG_UDP__
#define DEBUG_PRINT_NET(args...) SysLog(args)
#undef PIT_SEM_RATE
#define PIT_SEM_RATE 5
#else
#define DEBUG_PRINT_NET(args...)
#endif

//APPNAME Def
#ifdef RED
static const __attribute__((unused)) int test = 0;
#define APPNAME "GENVII Autopilot RED";
#endif
#ifdef GREEN
static const __attribute__((unused)) int test = 0;
#define APPNAME "GENVII Autopilot GREEN";
#endif

 /* DEFINITIONS_H_ */
