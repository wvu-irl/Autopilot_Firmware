/*
 * PWM.h
 *
 *  Created on: Aug 3, 2016
 *  Author: Scott Harper
 *
 */
#ifndef PWM_CLASS_H
#define PWM_CLASS_H

#include "../../Definitions.h"

#include <ucos.h>
#include <startnet.h>
#include <basictypes.h>
#include <sim.h>
#include <pins.h>

#define P_INIT 0
#define P_START 0

class PWM {
	public:
		int pwmChannelInput;
		uint16_t frequency;
		uint16_t PWM_lim;

		uint16_t LDOK;

		int module_no,pwm_num;
		uint16_t pwmr;

		PWM(int pwmChannelInput, int freq);

		 inline void setPWM(uint16_t PWMval){
			double setPWMVal = ((double)(PWMval* 0.00057675 + 0.91439)*0.001) * frequency *PWM_lim;
		 	sim1.mcpwm.sm[module_no].val[pwm_num]= setPWMVal;
		 	pwmr = sim1.mcpwm.mcr;
		 	sim1.mcpwm.mcr |= LDOK;
		 }

	private:
};
#endif
