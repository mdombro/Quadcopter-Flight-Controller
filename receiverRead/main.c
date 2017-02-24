#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "driverlib/fpu.h"
#include "utils/uartstdio.h"

// Converts timer clock ticks to us
// 40 for 40Mhz
#define CLOCK_PERIOD 1.0/40.0

#define AILERON_MAX 1890
#define AILERON_MIN 975
#define ELEVATOR_MAX 1928
#define ELEVATOR_MIN 991
#define THROTTLE_MAX 1947
#define THROTTLE_MIN 958  //958
#define RUDDER_MAX 1966
#define RUDDER_MIN 1028
#define EMERGENCY_MAX 1909
#define EMERGENCY_MIN 1033

//******************************************************************************************
//
//   Variables declaration
//
//******************************************************************************************
volatile uint32_t aileron, elevator, throttle, rudder, emergency, aileron_start, elevator_start, throttle_start, rudder_start, emergency_start;
volatile uint8_t aile_on, ele_on, thro_on, rud_on, emerg_on;
volatile bool EMERGENCY = false;

int32_t Map(int32_t in, int32_t inMin, int32_t inMax, int32_t outMin, int32_t outMax);

void RxCapture(void) {
	// Interrupt to handle receiver decoding
	// Each if statement will start timer counting on the rising edge (with some checks)
	// On falling edge for that pin timer is stopped and the value asessed
	// First channel is emergency stop - if its over a certain amount of time a global SYSTEM_OFF flag is set
	// Other channels are proportional channels
	// Will be scaled to whatever value is necessary for the application
	if (GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_2) == GPIO_PIN_2) {
		GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_2);  // Clear interrupt flag
		// Aileron
		aile_on = 1;
		ele_on = 0;
		thro_on = 0;
		rud_on = 0;
		emerg_on = 0;
		aileron_start = TimerValueGet(TIMER0_BASE, TIMER_A);
	}
	else if (GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_3) == GPIO_PIN_3) {
		GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_3);  // Clear interrupt flag
		// elevator
		aile_on = 0;
		ele_on = 1;
		thro_on = 0;
		rud_on = 0;
		emerg_on = 0;
		elevator_start = TimerValueGet(TIMER0_BASE, TIMER_A);
	}
	else if (GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_4) == GPIO_PIN_4) {
		GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_4);  // Clear interrupt flag
		// throttle
		aile_on = 0;
		ele_on = 0;
		thro_on = 1;
		rud_on = 0;
		emerg_on = 0;
		throttle_start = TimerValueGet(TIMER0_BASE, TIMER_A);
	}
	else if (GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_0) == GPIO_PIN_0) {
		GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_0);  // Clear interrupt flag
		// rudder
		aile_on = 0;
		ele_on = 0;
		thro_on = 0;
		rud_on = 1;
		emerg_on = 0;
		rudder_start = TimerValueGet(TIMER0_BASE, TIMER_A);
	}
	else if (GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_1) == GPIO_PIN_1) {
		GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_1);  // Clear interrupt flag
		// emergency
		aile_on = 0;
		ele_on = 0;
		thro_on = 0;
		rud_on = 0;
		emerg_on = 1;
		emergency_start = TimerValueGet(TIMER0_BASE, TIMER_A);
	}
	else {
		//TimerDisable(TIMER0_BASE, TIMER_A);
		if (aile_on) {
			aile_on = 0;
			ele_on = 0;
			thro_on = 0;
			rud_on = 0;
			emerg_on = 0;
			aileron = (TimerValueGet(TIMER0_BASE, TIMER_A)-aileron_start)*CLOCK_PERIOD;
			aileron = Map(aileron, AILERON_MIN, AILERON_MAX, -50, 50);
			GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_2);  // Clear interrupt flag
		}
		if (ele_on) {
			aile_on = 0;
			ele_on = 0;
			thro_on = 0;
			rud_on = 0;
			emerg_on = 0;
			elevator = (TimerValueGet(TIMER0_BASE, TIMER_A)-elevator_start)*CLOCK_PERIOD;
			elevator = Map(elevator, ELEVATOR_MIN, ELEVATOR_MAX, -50, 50);
			GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_3);  // Clear interrupt flag
		}
		if (thro_on) {
			aile_on = 0;
			ele_on = 0;
			thro_on = 0;
			rud_on = 0;
			emerg_on = 0;
			throttle = (TimerValueGet(TIMER0_BASE, TIMER_A)-throttle_start)*CLOCK_PERIOD;
			throttle = Map(throttle, THROTTLE_MIN, THROTTLE_MAX, 0, 100);
			GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_4);  // Clear interrupt flag
		}
		if (rud_on) {
			aile_on = 0;
			ele_on = 0;
			thro_on = 0;
			rud_on = 0;
			emerg_on = 0;
			rudder = (TimerValueGet(TIMER0_BASE, TIMER_A)-rudder_start)*CLOCK_PERIOD;
			rudder = Map(rudder, RUDDER_MIN, RUDDER_MAX, -50, 50);
			GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_0);  // Clear interrupt flag
		}
		if (emerg_on) {
			aile_on = 0;
			ele_on = 0;
			thro_on = 0;
			rud_on = 0;
			emerg_on = 0;
			emergency = (TimerValueGet(TIMER0_BASE, TIMER_A)-emergency_start)*CLOCK_PERIOD;
			if (emergency > 1500) EMERGENCY = true;
			else EMERGENCY = false;
			//emergency = Map(emergency, EMERGENCY_MIN, EMERGENCY_MAX, 0, 10);
			GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_1);  // Clear interrupt flag
		}
	}
}

void
ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 921600, 16000000);
}

int main(void)
{
	uint32_t ui32Period;

	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);

	// GPIO Port E setup
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	SysCtlDelay(3);
	GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_INT_PIN_2 | GPIO_INT_PIN_3 | GPIO_INT_PIN_4 | GPIO_INT_PIN_0 | GPIO_INT_PIN_1);

	ConfigureUART();

	// Interrupt setup
	GPIOIntDisable(GPIO_PORTE_BASE, GPIO_INT_PIN_2 | GPIO_INT_PIN_3 | GPIO_INT_PIN_4 | GPIO_INT_PIN_0 | GPIO_INT_PIN_1);
	GPIOIntClear(GPIO_PORTE_BASE, GPIO_INT_PIN_2 | GPIO_INT_PIN_3 | GPIO_INT_PIN_4 | GPIO_INT_PIN_0 | GPIO_INT_PIN_1);
	GPIOIntEnable(GPIO_PORTE_BASE, GPIO_INT_PIN_2 | GPIO_INT_PIN_3 | GPIO_INT_PIN_4 | GPIO_INT_PIN_0 | GPIO_INT_PIN_1);
	GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_INT_PIN_2 | GPIO_INT_PIN_3 | GPIO_INT_PIN_4 | GPIO_INT_PIN_0 | GPIO_INT_PIN_1, GPIO_BOTH_EDGES);
	GPIOIntRegister(GPIO_PORTE_BASE, RxCapture);

	// Timer setup
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	SysCtlDelay(3);
	TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC_UP);
	TimerEnable(TIMER0_BASE, TIMER_A);
	//TimerDisable(TIMER0_BASE, TIMER_A);

	IntEnable(INT_GPIOE);
	IntMasterEnable();


	aile_on = 0;
	ele_on = 0;
	thro_on = 0;
	rud_on = 0;
	emerg_on = 0;

	uint16_t count = 0;

	while(1)
	{
		count++;
		if (count == 50000) {
			count = 0;
			UARTprintf("%3d, %3d, %3d, %3d, %3d\n", aileron, elevator, throttle, rudder, EMERGENCY);
		}
	}
}

int32_t Map(int32_t in, int32_t inMin, int32_t inMax, int32_t outMin, int32_t outMax) {
	return ((in - inMin)*(outMax - outMin) / (inMax - inMin)) + outMin;
}
