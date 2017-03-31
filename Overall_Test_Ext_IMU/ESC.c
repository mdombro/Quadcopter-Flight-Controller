/*
 * ESC.c
 *
 *  Created on: Feb 21, 2017
 *      Author: Matthew
 */

#include "ESC.h"

int32_t Map_f_in_int_out(float in, float inMin, float inMax, float outMin, float outMax);
int32_t map(int32_t in, int32_t inMin, int32_t inMax, int32_t outMin, int32_t outMax);

void writePWM1(int32_t spd) {
	int32_t val = Map_f_in_int_out(spd, 0.0, 100.0, PWM_MIN_TICKS, PWM_MAX_TICKS);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, val);
}

void writePWM2(int32_t spd) {
	int32_t val = Map_f_in_int_out(spd, 0.0, 100.0, PWM_MIN_TICKS, PWM_MAX_TICKS);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, val);
}

void writePWM3(int32_t spd) {
	int32_t val = Map_f_in_int_out(spd, 0.0, 100.0, PWM_MIN_TICKS, PWM_MAX_TICKS);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, val);
}

void writePWM4(int32_t spd) {
	int32_t val = Map_f_in_int_out(spd, 0.0, 100.0, PWM_MIN_TICKS, PWM_MAX_TICKS);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, val);
}


// Even though identical to Map in main, this is the special map for PWM work
int32_t map(int32_t in, int32_t inMin, int32_t inMax, int32_t outMin, int32_t outMax) {
	return ((in - inMin)*(outMax - outMin) / (inMax - inMin)) + outMin;
}

int32_t Map_f_in_int_out(float in, float inMin, float inMax, float outMin, float outMax) {
	return (int32_t)( ((in - inMin)*(outMax - outMin) / (inMax - inMin)) + outMin );
}

void ConfigurePWM() {
	// enable the GPIO port F pins (PF1 is the target LED)
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
	GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_6);
	// Enable PWM5 which is on PF1, an LED pin
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);

	GPIOPinConfigure(GPIO_PA6_M1PWM2);
	GPIOPinConfigure(GPIO_PF1_M1PWM5);
	GPIOPinConfigure(GPIO_PF2_M1PWM6);
	GPIOPinConfigure(GPIO_PF3_M1PWM7);
	GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
	GPIOPinTypePWM(GPIO_PORTA_BASE, GPIO_PIN_6);


    SysCtlPWMClockSet(SYSCTL_PWMDIV_8);  // ensure freq division matches defines PWM_MIN_TICKS and PWM_MAX_TICKS at top
	// Configure the PWM generator for count down mode with immediate updates
	// to the parameters.
	//
	// Gen 2 has M1PWM4 and M1PWM5
    PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
	PWMGenConfigure(PWM1_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
	PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
	//
	// Set the period. For a 100 Hz frequency, the period = 1/100, or 10
	// mseconds. For a 625 kHz clock, this translates to 6250-1 clock ticks.
	// Use this value to set the period.
	//
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, PWM_PERIOD_TICKS );
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, PWM_PERIOD_TICKS );
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, PWM_PERIOD_TICKS );
	//
	// Set the pulse width of PWM_OUT_5 for a 1ms pulse width.
	//
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, 5000);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, 5000);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, 5000);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, 5000);
	//
	// Start the timers in generator 2.
	//
	PWMGenEnable(PWM1_BASE, PWM_GEN_1);
	PWMGenEnable(PWM1_BASE, PWM_GEN_2);
	PWMGenEnable(PWM1_BASE, PWM_GEN_3);
	//
	// Enable the outputs.
	//
	PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT | PWM_OUT_5_BIT | PWM_OUT_6_BIT | PWM_OUT_7_BIT, true);   //  | PWM_OUT_1_BIT
}
