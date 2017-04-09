#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/fpu.h"
#include "utils/uartstdio.h"
#include "driverlib/timer.h"


uint8_t RxBuffer[20];

//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
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
    UARTStdioConfig(0, 115200, 16000000);
}

void UARTInt(void) {
	uint32_t terminationPos = UARTPeek('\r');

	if (terminationPos == 11) {
		UARTgets(&RxBuffer, 12);
	}

	UARTprintf("%d  %d  %d  %d  %d  %d  %d  %d  %d  %d  %d  %d\n", RxBuffer[0], RxBuffer[1], RxBuffer[2], RxBuffer[3], RxBuffer[4], RxBuffer[5], RxBuffer[6], RxBuffer[7], RxBuffer[8], RxBuffer[9], RxBuffer[10], RxBuffer[11]);
}

int main(void) {
	// Setup the system clock to run at 40 Mhz from PLL with crystal reference
	SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

	//
	// Initialize the UART.
	//
	ConfigureUART();
	
	//**********************************************************
	//
	// Pulsed Light Time Interrupt Config
	//
	//**********************************************************
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
	TimerConfigure(TIMER3_BASE, TIMER_CFG_PERIODIC);
	uint32_t ui32Period3 = SysCtlClockGet() / 100;
	TimerLoadSet(TIMER3_BASE, TIMER_A, ui32Period3 - 1);

	IntPrioritySet(INT_TIMER5A, 0b11100000);
	IntEnable(INT_TIMER5A);
	TimerIntEnable(TIMER5_BASE, TIMER_TIMA_TIMEOUT);
	IntMasterEnable();

	return 0;
}
