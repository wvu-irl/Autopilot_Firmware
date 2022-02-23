/*
 * PWM.cpp
 *
 *  Created on: Aug 3, 2016
 *      Author: Scott Harper
 *
 *      Update 11/17/2017: Added NANO54415 Capatibility
 */

#include "PWM.h"
#include "../../definitions.h"

static uint16_t CR2= 0xE000; //Debug enabled, Wait enabled, output A&B are independent,
//#define CR1 0x0444 //400 Hz Full cycle reloaded enabled, PWN clock fclk/16, Load mode selected immediatly after LDOK being set.
//#define CR1 0x0474 //50 Hz
static uint16_t CR1=0x0404; //Vanilla CR1
static uint16_t OCR=0x0000; //10-8: output of A&B&x not inverted, 5-4 3-2 1-0: output of A&B&x forced to 0 prior to output polarity control
static uint16_t DISMAP=0x0000; //Fault disable mapping register. PWM fault pin has no effect on x B and A
static uint16_t OUTEN=0x0000;  //Output enable register. 11-8 7-4 3-0: A B enabled, x not enabled.
//Master control register. 15-12: ingored by independent mode; 3-0 Load OK,
static uint16_t P_RUN=0x0000 ; //PWM generator enabled.

 PWM::PWM (int pwmChannelInput, int freq){

		module_no=999; pwm_num=999;
		pwmr=0;
		frequency = freq;


 	switch(pwmChannelInput){
 		case 0xA0:

#ifdef MOD54415
 		J2[25].function(2);//PWMA0
#endif
#ifdef NANO54415_PIN_CONSTANT
 		Pins[31].function(PIN_31_PWM_A0);//PWMA0
#endif
 		module_no=0;
 		pwm_num=3;
 		OUTEN=OUTEN | 0x0100;
 		LDOK=LDOK | 0x0001;
 		P_RUN=P_RUN | 0x0100;
 		break;

 		case 0xA1:
#ifdef MOD54415
 		J2[30].function(2);//PWMA1
#endif
#ifdef NANO54415_PIN_CONSTANT
 		Pins[15].function(PIN_15_PWM_A1);//PWMA1
#endif
 		module_no=1;
 		pwm_num=3;
 		OUTEN=OUTEN | 0x0200;
 		LDOK=LDOK | 0x0002;
 		P_RUN=P_RUN | 0x0200;
 		break;

 		case 0xA2:
#ifdef MOD54415
 		J2[35].function(2);//PWMA2
#endif
#ifdef NANO54415_PIN_CONSTANT
 		Pins[37].function(PIN_37_PWM_A2);//PWMA2
#endif
 		module_no=2;
 		pwm_num=3;
 		OUTEN=OUTEN | 0x0400;
 		LDOK=LDOK | 0x0004;
 		P_RUN=P_RUN | 0x0400;
 		break;

 		case 0xA3:
#ifdef MOD54415
 		J2[31].function(2);//PWMA3
#endif
#ifdef NANO54415_PIN_CONSTANT
 		Pins[13].function(PIN_13_PWM_A3);//PWMA3
#endif
 		module_no=3;
 		pwm_num=3;
 		OUTEN=OUTEN | 0x0800;
 		LDOK=LDOK | 0x0008;
 		P_RUN=P_RUN | 0x0800;
 		break;

 		case 0xB0:
#ifdef MOD54415
 		J2[27].function(2);//PWMB0;
#endif
#ifdef NANO54415_PIN_CONSTANT
 		Pins[33].function(PIN_33_PWM_B0);//PWMB0
#endif
 		module_no=0;
 		pwm_num=5;
 		OUTEN=OUTEN | 0x0010;
 		LDOK=LDOK | 0x0001;
 		P_RUN=P_RUN | 0x0100;
 		break;

 		case 0xB1:
#ifdef MOD54415
 		J2[40].function(2);//PWMB1
#endif
#ifdef NANO54415_PIN_CONSTANT
 		Pins[39].function(PIN_39_PWM_B1);//PWMB1
#endif
 		module_no=1;
 		pwm_num=5;
 		OUTEN=OUTEN | 0x0020;
 		LDOK=LDOK | 0x0002;
 		P_RUN=P_RUN | 0x0200;
 		break;

 		case 0xB2:
#ifdef MOD54415
 		J2[28].function(2);//PWMB2
#endif
#ifdef NANO54415_PIN_CONSTANT
 		Pins[35].function(PIN_35_PWM_B2);//PWMB2
#endif
 		module_no=2;
 		pwm_num=5;
 		OUTEN=OUTEN | 0x0040;
 		LDOK=LDOK | 0x0004;
 		P_RUN=P_RUN | 0x0400;
 		break;

 		case 0xB3:
#ifdef MOD54415
 		J2[19].function(2);//PWMB3
#endif
#ifdef NANO54415_PIN_CONSTANT
 		Pins[16].function(PIN_16_PWM_B3);//PWMB3
#endif
 		module_no=3;
 		pwm_num=5;
 		OUTEN=OUTEN | 0x0080;
 		LDOK=LDOK | 0x0008;
 		P_RUN=P_RUN | 0x0800;
 		break;
 	}

 	int frequency_divider=1;

 	if(frequency<=59){
 		CR1=CR1|(7<<4);
 		frequency_divider=128;

 	}
 	else if(frequency>59 && frequency<=100){
 		CR1=CR1|(6<<4);
 		frequency_divider=64;
 	}
 	else if(frequency>100 && frequency<=200){
 		CR1=CR1|(5<<4);
 		frequency_divider=32;
 	}
 	else{
 		CR1=CR1|(4<<4);
 		frequency_divider=16;
 	}

 	PWM_lim = 125000000L/(frequency*frequency_divider)+1;

 	//PWM Registers
 	sim1.mcpwm.sm[module_no].cr2=CR2;
 	sim1.mcpwm.sm[module_no].cr1=CR1;
 	sim1.mcpwm.sm[module_no].ocr=OCR;
 	sim1.mcpwm.sm[module_no].dismap=DISMAP;
 //Edge Aligned PWM
 	sim1.mcpwm.sm[module_no].init=P_INIT;
 	sim1.mcpwm.sm[module_no].val[1]=PWM_lim;
 	sim1.mcpwm.sm[module_no].val[pwm_num-1]=P_START;
 	sim1.mcpwm.sm[module_no].val[pwm_num]=PWM_lim*frequency*0.0009;

 //enabling PWM Outputs
 	sim1.mcpwm.outen=OUTEN;
 //starting PWM
 	pwmr=sim1.mcpwm.mcr;
 	sim1.mcpwm.mcr=LDOK;
 	sim1.mcpwm.mcr |=P_RUN;
 }




