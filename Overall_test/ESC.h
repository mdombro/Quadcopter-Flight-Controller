/*
 * ESC.h
 *
 *  Created on: Feb 21, 2017
 *      Author: Matthew
 */

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"

#ifndef ESC_H_
#define ESC_H_

//----------------------- PWM defines ----------------------------------------------------------------//
// If there are problems check:
//		- Is SysCtlPWMClockSet() getting the right clock divider configuration for given constants?
// 		- For ANY change in the overall PWM module clock, the constants MUST be re-calculated
#define PWM_CLOCK_FREQ 5000000
#define PWM_FREQ 500
#define PWM_PERIOD_TICKS (PWM_CLOCK_FREQ/PWM_FREQ)-1
#define PWM_MIN_TICKS 5000  		// valid for PWM_CLOCK_FREQ of 5MHz only
#define PWM_MAX_TICKS 9999


// --------------------- Function Prototype Definitions ----------------------------------------------//
void ConfigurePWM();   // configure PWM module 1, output 5 on PF1
int32_t map(int32_t in, int32_t inMin, int32_t inMax, int32_t outMin, int32_t outMax);
void writePWM1(int32_t spd);
void writePWM2(int32_t spd);
void writePWM3(int32_t spd);
void writePWM4(int32_t spd);



#endif /* ESC_H_ */
