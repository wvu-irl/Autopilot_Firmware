/******************************************************************************/
/* SMART Robot                                                    			  */
/* Author: Yu Gu, Jason Gross                           					  */
/* Version: 1.0						                          				  */
/* Target CPU: MCF5213                                                        */
/* Last update: July 24, 2013                                                 */
/******************************************************************************/

/******************************************************************************/
/* Includes                                                                   */
/******************************************************************************/
#include "predef.h"
#include <stdio.h>
#include <ctype.h>
#include <basictypes.h>
#include <serialirq.h>
#include <system.h>
#include <constants.h>
#include <ucos.h>
#include <cfinter.h>
#include <utils.h>
#include <a2d.h>
#include <pins.h>
#include <math.h>
#include <..\MOD5213\system\sim5213.h>
#include <smarttrap.h> // NetBurner Smart Trap utility

/******************************************************************************/
/* Definitions                                                                */
/******************************************************************************/

#define UART0           0
#define UART1           1
#define UART2           2

// Stack Definitions
//#define INPUT_PACKET_SIZE 13
//#define INPUT_HEADER_SIZE 5
//#define INPUT_TASK_STK_SIZE (384)

// memory map of ADIS 16405 IMU
#define SUPPLY_OUT	0x0200
#define XGYRO_OUT	0x0400
#define YGYRO_OUT	0x0600
#define ZGYRO_OUT	0x0800
#define XACCL_OUT	0x0A00
#define YACCL_OUT	0x0C00
#define ZACCL_OUT	0x0E00
#define XMAG_OUT	0x1000
#define YMAG_OUT	0x1200
#define ZMAG_OUT	0x1400
#define TEMP_OUT	0x1600

// Use the #defines below to load sim.timer.dtrr register.
// Value=( ScaledClock/TargetFrequency) + 1
// where ScaledClock is (66355200 / Prescaler ), Prescaler is 16
// #define DMA_FIFTY_HZ    82945
// Time between PWM channels (1.0 ms)
// #define DMA_PWM_LOW     4147

// Assign Pins to PWM and LED channels
//#define PWM_CH0         8
//#define PWM_CH1         7
//#define PWM_CH2         6
//#define PWM_CH3         5
//#define PWM_CH4         4
//#define PWM_CH5         33
#define LED_CH1         21
#define LED_CH2         22

// Reorder NB AD Ch to Boards A/D Ch
//#define BATT          1
//#define ADCH0         4
//#define ADCH1         5
//#define ADCH2         6
//#define ADCH3         7
//#define ADCH4         3
//#define ADCH5         0
// int ADOrder[8] = { 1, 4, 5, 6, 7, 3, 0, 2 };
// unsigned int AD_SPI_DATA[4];

// PIT Channels
#define PIT_CH1         1

extern "C" {
void UserMain(void *pd);
// Set up the 5213 interrupt controller
}
extern void SetIntc(long func, int vector, int level, int prio);

/******************************************************************************/
/* Global variables                                                           */
/******************************************************************************/

/********** Name for development tools to identify this application **********/
const char *AppName = "ADIS_IMU_COM";
unsigned char Output_Packet_Count = 0;

/************************************ PIT ************************************/
volatile DWORD pitr_count;
volatile DWORD RF_count;		// Counter for the range finder
char CS = 0;

/******************************* IMU Data ************************************/
volatile short int adis_data[10] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 };
short int adis_sum[10];
unsigned int spi_data[2];
unsigned int imu_spi_data[10];
short int imu_data[10];

/******************************* A/D Data ************************************/
unsigned int ADcounts[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
unsigned int ADC[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
unsigned int ADsum[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

/******************************* Controller Commands Data Packet Input***************************/
// DWORD INPUTTaskStack[INPUT_TASK_STK_SIZE];
// unsigned char Input_Packet[INPUT_PACKET_SIZE];
// int Input_Packet_Count = 0;
// char Input_Data[INPUT_PACKET_SIZE];
// char Range_Data[INPUT_PACKET_SIZE];
// int Input_Data_Ready = 0;
// unsigned char Input_Packet_Header[INPUT_HEADER_SIZE] = { 68, 73, 83, 84, 58 }; //ASCII DIST:
// unsigned char Input_Check_Sum = 0;

/*************************** Variables for Implementing of Camera Switch *************************/
unsigned int Packet_Count = 0;
// volatile WORD PWM_READ[3];
unsigned long counter = 0;

/******************************************************************************/
/* INIT: Pin Configuration                                           */
/******************************************************************************/
void InitPinClass() {
	// InitSerialPorts()
	Pins[2].function(PIN2_UART0_RX);	// COM 0 RX Software Download
	Pins[3].function(PIN3_UART0_TX);	// COM 0 TX
	Pins[29].function(PIN29_UART1_RX); 	// COM 1 RX Talk to Laptop
	Pins[30].function(PIN30_UART1_TX); 	// COM 1 TX Talk to Laptop
	Pins[31].function(PIN31_UART2_RX); 	// COM 2 RX Reserved
	Pins[32].function(PIN32_UART2_TX); 	// COM 2 TX Reserved

	// Pins for LED indicators
	Pins[21].function(PIN21_GPIO); 		// LED  1
	Pins[22].function(PIN22_GPIO); 		// LED  2

	// Pins for Switches
	Pins[24].function(PIN24_GPIO); 		// Switch  1
	Pins[23].function(PIN23_GPIO); 		// Switch  2

	// Configure the A2D pins as analog inputs
	Pins[13].function(PIN13_AN0); 		// A/D Ch 1
	Pins[12].function(PIN12_AN1); 		// A/D Ch 2
	Pins[11].function(PIN11_AN2); 		// Front Range Finder
	Pins[14].function(PIN14_AN3); 		// Front Left Range Finder
	Pins[15].function(PIN15_AN7); 		// Front Right Range Finder
	Pins[16].function(PIN16_AN6); 		// Right Range Finder
	Pins[17].function(PIN17_AN5); 		// Back Range Finder
	Pins[18].function(PIN18_AN4); 		// Left Range Finder

	// InitServoControl()
	Pins[8].function(PIN8_GPIO); 		// PWM  Ch 1
	Pins[7].function(PIN7_GPIO); 		// PWM  Ch 2
	Pins[6].function(PIN6_GPIO); 		// PWM  Ch 3
	Pins[5].function(PIN5_GPIO); 		// PWM  Ch 4
	Pins[4].function(PIN4_GPIO); 		// PWM  Ch 5
	Pins[35].function(PIN35_GPIO); 		// PWM  Ch 6
	Pins[33].function(PIN33_GPIO); 		// PWM  Ch 7

	// InitPPM_Read()
	Pins[25].function(PIN25_GPT3); // PPM Read 0
	Pins[26].function(PIN26_GPT2); // PPM Read 1

	// InitSPI
	Pins[34].function(PIN34_QSPI_CS1); 	// ADIS Chip Select
	Pins[36].function(PIN36_QSPI_DOUT); // ADIS/SPI
	Pins[37].function(PIN37_QSPI_DIN); 	// ADIS/SPI
	Pins[38].function(PIN38_QSPI_CLK); 	// ADIS/SPI Clock
}

/******************************************************************************/
/* INIT: Serial Port                                                          */
/******************************************************************************/
void InitSerialPorts() {
	InitUart(UART2, 115200, 1, 8, eParityNone);
	InitUart(UART1, 115200, 1, 8, eParityNone);
	InitUart(UART0, 115200, 1, 8, eParityNone);
	assign_stdio(UART0);
}

/******************************************************************************/
/* INIT: SPI Port for ADIS16405 IMU                                           */
/******************************************************************************/
void Init_IMU_SPI() {
	int i = 0;
	sim.spi.qmr = 0xC380;	// 0x00_0340 -> 0x00_0341 - QSPI Mode Register, 0.258MHz Baud Rate
	//IMU Data
	sim.spi.qar = 0x0022;	// 0x00_0350 -> 0x00_0351 - QSPI Address Register, select the first command RAM entry
	for (i = 0; i < 10; i++) {
		sim.spi.qdr = 0x7d00;	// 0x00_0354 -> 0x00_0355 - QSPI Data Register
	}
//	sim.spi.qdr = 0x7d00;
//	sim.spi.qdr = 0x7d00;
//	sim.spi.qdr = 0x7d00;
//	sim.spi.qdr = 0x7d00;
//	sim.spi.qdr = 0x7d00;
//	sim.spi.qdr = 0x7d00;
//	sim.spi.qdr = 0x7d00;
//	sim.spi.qdr = 0x7d00;
//	sim.spi.qdr = 0x7d00;

	sim.spi.qar = 0x0002; 	//select the first transmit ram entry
	//IMU Data
	sim.spi.qdr = YACCL_OUT;
	sim.spi.qdr = ZACCL_OUT;
	sim.spi.qdr = XGYRO_OUT;
	sim.spi.qdr = YGYRO_OUT;
	sim.spi.qdr = ZGYRO_OUT;
	sim.spi.qdr = XMAG_OUT;
	sim.spi.qdr = YMAG_OUT;
	sim.spi.qdr = ZMAG_OUT;
	sim.spi.qdr = TEMP_OUT;
	sim.spi.qdr = XACCL_OUT;

	sim.spi.qwr = 0x7B02;	// 0x00_0348 -> 0x00_0349 - QSPI Wrap Register, after getting to the 10 piece of data return to zero
	sim.spi.qdlyr = (0x03FF | 0x8000);	// 0x00_0344 -> 0x00_0345 - QSPI Delay Register
}

/******************************************************************************/
/* INTERRUPT: Read Data from the ADIS16405 IMU                                */
/******************************************************************************/
//INTERRUPT( QSPI, 0x2500)
void Read_IMU_SPI() {

	sim.spi.qir = 0x0001;	//0x00_034C -> 0x00_034D - QSPI Interrupt Register
	sim.spi.qar = 0x0012; 	//select the first receiver entry
	int i = 0;
	for (i = 0; i < 10; i++) {
		imu_spi_data[i] = (unsigned int) (sim.spi.qdr);
	}
	int j = 0;
	for (j = 0; j < 9; j++) {
		adis_data[j] = (short int) ((imu_spi_data[j] & 0x3FFF) << 2) / 4;
	}
	adis_data[9] = imu_spi_data[9];
	int k = 0;
	for (k = 0; k < 10; k++) {
		adis_sum[k] = adis_sum[k] + adis_data[k];
	}
	// sim.spi.qir=0x0100;
	//sim.spi.qdlyr=(0x10d4|0x8000);
}

void Average_IMU(){
	int i=0;
	for (i=0; i<10; i++)
	{
		imu_data[i]=adis_sum[i]/4;
		adis_sum[i]=0;
	}
}

void A2D() {
	int i, j;
	for (i = 0; i < 8; i++) {
		//int temp =ADOrder[i];
//		ADcounts[i] = ReadA2DResult(ADOrder[i]) >> 3;
		ADcounts[i] = ReadA2DResult(i) >> 3;
	}
	for (j = 0; j < 8; j++) {
		ADsum[j] = ADsum[j] + ADcounts[j];
	}

}

void Average_A2D() {
	int j;
	for (j = 0; j < 8; j++) {
		ADC[j] = ADsum[j] / 32;
		ADsum[j] = 0;
	}
}

/******************************************************************************/
/*Assemble The Data Packet                                                    */
/******************************************************************************/
void Assemble_Packet() {

	int k = 0;
	int n = 0;
	int o = 0;
	int l = 0;
	// Blink LED for .5 second
	if (Packet_Count == 0) {
		Pins[LED_CH1] = 0;
		Pins[LED_CH2] = 1;
	}
	if (Packet_Count == 100) {
		Pins[LED_CH1] = 1;
		Pins[LED_CH2] = 0;
	}
	if (Packet_Count == 200) {
		Packet_Count = 0;
		Pins[LED_CH1] = 0;
		Pins[LED_CH2] = 1;
	}

	Packet_Count = Packet_Count + 1;
    counter=counter+1;
	char Output_Packet[50];
	Output_Packet[0]=65;												// First Header (ASCII 'A')
	Output_Packet[1]=90;												// Second Header (ASCII 'Z')
	Output_Packet[2] = (counter & 0x00FF);								// 8-bit Counter
	for (k = 0; k < 10; k++) {
		Output_Packet[((k * 2) + 3)] = (imu_data[k] & 0xFF00) >> 8; 	// IMU MSB
		Output_Packet[((k * 2) + 4)] = (imu_data[k] & 0x00FF); 			// IMU LSB
	}
	for (n = 0; n < 8; n++) {
		Output_Packet[((n * 2) + 23)] = (ADC[n] & 0xFF00) >> 8; 		// A/D MSB
		Output_Packet[((n * 2) + 24)] = (ADC[n] & 0x00FF); 				// A/D LSB
	}
	if(Pins[24])
		Output_Packet[39] = 1;											// If switch #1 is on
	else
		Output_Packet[39] = 0;
	if(Pins[23])
		Output_Packet[39] += 2;											// If switch #2 is on
	Output_Packet[40] = 67;												// Reserved
	Output_Packet[41] = 68;												// Reserved
	Output_Packet[42] = 69;												// Reserved
	Output_Packet[43] = 70;												// Reserved
	Output_Packet[44] = 71;												// Reserved
	Output_Packet[45] = 72;												// Reserved
	Output_Packet[46] = 73;												// Reserved
	Output_Packet[47] = 74;												// Reserved
	Output_Packet[48] = 75;												// Reserved

	Output_Packet[49] =0;
	for (o = 0; o < 49; o++) {
		Output_Packet[49] += Output_Packet[o]; 							// Checksum
	}

	for (l = 0; l < 50; l++) {
		char temp = Output_Packet[l];
		writechar(1, temp);
	}

}

/******************************************************************************/
/* INTERRUPT: PIT1 ISR 50 Hz, Main routine for the Controller!                */
/******************************************************************************/
INTERRUPT( my_pitr_func, 0x2600 )
{
	pitr_count++;
	RF_count++;
	if (RF_count==80) {				//Reset every 50ms
		RF_count=0;
		Pins[8]=1; }					//Activate all the range finders
	else {
		Pins[8]=0;
	}
	WORD tmp = sim.pit[1].pcsr; 	// Use PIT 1
	tmp &= 0xFF0F; 					// Bits 4-7 cleared
	tmp |= 0x0F; 					// Bits 0-3 set
	sim.pit[1].pcsr = tmp;
	if (pitr_count == 26||pitr_count== 28||pitr_count == 30||pitr_count== 32) {
			Read_IMU_SPI();
			Init_IMU_SPI();
	}
	if (pitr_count == 32) {
		Average_A2D();
		Average_IMU();
		Assemble_Packet();
		pitr_count = 0;
	}
	A2D();
}

/******************************************************************************/
/* INIT: PIT1 Initialization                                                  */
/******************************************************************************/
void InitPIT1(WORD clock_interval, BYTE pcsr_pre /*table 173 bits 811 */) {
	WORD tmp;
	// Populate the interrupt vector in the interrupt controller
	SetIntc((long) &my_pitr_func, 55 + PIT_CH1, 2 /* IRQ 2 */, 1);
	sim.pit[PIT_CH1].pmr = clock_interval; 	// Set the PIT modulus value
	tmp = pcsr_pre;
	tmp = (tmp << 8) | 0x0F;
	sim.pit[PIT_CH1].pcsr = tmp;
	// Set the system clock divisor to 2
}

/******************************************************************************/
/* USERMAIN: Main function                                                    */
/******************************************************************************/
void UserMain(void *pd) {
	EnableSmartTraps(); 		// enable smart trap utility
	OSChangePrio(MAIN_PRIO);
	InitSerialPorts(); 			// Init UART ports
	InitPinClass(); 			// Assign function to each Pin
	EnableAD();
	Init_IMU_SPI();
	iprintf("IRL SMART SENSOR INTERFACE FIRMWARE V1.1");
	// Waiting 20735 counts with an internal bus clock divisor of 32 =50Hz
	InitPIT1(20735 /* Wait 20735 clocks */, 0 /*Divide by 32 from table 173*/);

	while (1) {
		OSTimeDly(TICKS_PER_SECOND / 5);
		//iprintf("%d,%d,%d,%d\r\n", AD_SPI_DATA[0],AD_SPI_DATA[1],AD_SPI_DATA[2],AD_SPI_DATA[3]);
		iprintf("IMU-->%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n", imu_data[0], imu_data[1], imu_data[2], imu_data[3], imu_data[4],imu_data[5],imu_data[6], imu_data[7], imu_data[8], imu_data[9]);
		iprintf("AD-->%d,%d,%d,%d,%d,%d,%d,%d\r\n", ADC[0], ADC[1], ADC[2], ADC[3], ADC[4],ADC[5],ADC[6], ADC[7]);

//		if(!Pins[24])
//			iprintf("Switch 1 is low");
	}
}
