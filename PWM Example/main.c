#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"

int main() {
	unsigned long i = 0;

	// set up the clock for 40MHz
	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);

	//Configure PWM Clock to match system
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

    // enable the GPIO port F pins (PF1 is the target LED)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	// Enable PWM5 which is on PF1, an LED pin
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);

	GPIOPinConfigure(GPIO_PF1_M1PWM5);
	GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1);

	//
	// Configure the PWM generator for count down mode with immediate updates
	// to the parameters.
	//
	// Gen 2 has M1PWM4 and M1PWM5
	PWMGenConfigure(PWM1_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

	//
	// Set the period. For a 40 Hz frequency, the period = 1/40, or 25
	// mseconds. For a 625 kHz clock, this translates to 1562 clock ticks.
	// Use this value to set the period.
	//
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, 15625);

	//
	// Set the pulse width of PWM_OUT_5 for a 25% duty cycle.
	//
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, 3900);
	

	//
	// Start the timers in generator 2.
	//
	PWMGenEnable(PWM1_BASE, PWM_GEN_2);

	//
	// Enable the outputs.
	//
	PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT, true);   //  | PWM_OUT_1_BIT

	while(1) {
		PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, i);
		i++;
		if (i == 15000) {i = 0;}
		SysCtlDelay(800000);
	}
}

