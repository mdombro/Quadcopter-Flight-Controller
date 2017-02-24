#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"

//----------------------- PWM defines ----------------------------------------------------------------//
// If there are problems check:
//		- Is SysCtlPWMClockSet() getting the right clock divider configuration for given constants?
// 		- For ANY change in the overall PWM module clock, the constants MUST be re-calculated
#define PWM_CLOCK_FREQ 625000
#define PWM_FREQ 450
#define PWM_PERIOD_TICKS (PWM_CLOCK_FREQ/PWM_FREQ)-1
#define PWM_MIN_TICKS 625  		// valid for PWM_CLOCK_FREQ of 625,000 Hz only
#define PWM_MAX_TICKS 1250
// ---------------------------------------------------------------------------------------------------//



// --------------------- Function Prototype Definitions ----------------------------------------------//
void ConfigureSystemClock();
void ConfigurePeripheralsAndGPIO();
void ConfigurePWM1O5();   // configure PWM module 1, output 5 on PF1
uint32_t PulseWidthMicroSeconds(uint16_t us);
uint32_t Map(uint32_t in, uint32_t inMin, uint32_t inMax, uint32_t outMin, uint32_t outMax);

// ---------------------------------------------------------------------------------------------------//



int main() {
//  ------------------- Local Variable declarations ---------------------------//
	uint16_t i = 1000;

// ----------------------------------------------------------------------------//

	ConfigureSystemClock();
	ConfigurePeripheralsAndGPIO();
	ConfigurePWM1O5();

	// wait for ESC to initialize
	SysCtlDelay(80000000);
	
	int8_t inc = 1;
	while(1) {
		SysCtlDelay(100000);
		PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, PulseWidthMicroSeconds(i));
		i+=inc;
		if (i == 2000)
			inc = -1;
		if (i == 1000)
			inc = 1;
	}
}


// ---------------------  Function Definitions -------------------------------------------------------//

uint32_t PulseWidthMicroSeconds(uint16_t us) {
	return Map(us, 1000, 2000, 625, 1250);
}

uint32_t Map(uint32_t in, uint32_t inMin, uint32_t inMax, uint32_t outMin, uint32_t outMax) {
	return ((in - inMin)*(outMax - outMin) / (inMax - inMin)) + outMin;
}

void ConfigureSystemClock() {
	// set up the clock for 40MHz
	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
	//Configure PWM Clock to match system
}

void ConfigurePeripheralsAndGPIO() {
    // enable the GPIO port F pins (PF1 is the target LED)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	// Enable PWM5 which is on PF1, an LED pin
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);

	GPIOPinConfigure(GPIO_PF1_M1PWM5);
	GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1);
}

void ConfigurePWM1O5() {
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);  // ensure freq division matches defines PWM_MIN_TICKS and PWM_MAX_TICKS at top
	// Configure the PWM generator for count down mode with immediate updates
	// to the parameters.
	//
	// Gen 2 has M1PWM4 and M1PWM5
	PWMGenConfigure(PWM1_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
	//
	// Set the period. For a 100 Hz frequency, the period = 1/100, or 10
	// mseconds. For a 625 kHz clock, this translates to 6250-1 clock ticks.
	// Use this value to set the period.
	//
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, PWM_PERIOD_TICKS );
	//
	// Set the pulse width of PWM_OUT_5 for a 1ms pulse width.
	//
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, PulseWidthMicroSeconds(1000));
	//
	// Start the timers in generator 2.
	//
	PWMGenEnable(PWM1_BASE, PWM_GEN_2);
	//
	// Enable the outputs.
	//
	PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT, true);   //  | PWM_OUT_1_BIT
}

