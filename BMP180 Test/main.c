// bmp180.c
//
//****************************************************************************************************
// Author:
// 	Nipun Gunawardena
//
// Credits:
//	Modified from the TivaWare 'bmp180.c' program. Thanks to adafruit for the coding sanity check
//	See https://github.com/adafruit/Adafruit_BMP085_Unified
//
// Requirements:
// 	Requires Texas Instruments' TivaWare.
//
// Description:
// 	Interface with Bosch BMP180 on SensorHub boosterpack
//
// Notes:
//	See datasheet for intermediate value naming conventions
//
//
//****************************************************************************************************


// Includes ------------------------------------------------------------------------------------------
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "inc/hw_ints.h"
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"

#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/i2c.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"

#include "utils/uartstdio.h"

#include "bmpLib.h"




// Defines -------------------------------------------------------------------------------------------
#define LED_RED GPIO_PIN_1
#define LED_BLUE GPIO_PIN_2
#define LED_GREEN GPIO_PIN_3



// Variables -----------------------------------------------------------------------------------------
uint32_t ui32Period;
// Create printing variable
uint32_t printValue[2];
// Initialize BMP180 and get calibration data
tBMP180 BmpSensHub;
tBMP180Cals BmpSensHubCals;

// Functions -----------------------------------------------------------------------------------------
void ConfigureUART(void){

	// Enable the peripherals used by UART
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

	// Set GPIO A0 and A1 as UART pins.
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

        // Configure UART clock using UART utils
        UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
        UARTStdioConfig(0, 115200, 16000000);
}

void ConfigureI2C1(bool fastMode){

	// Enable peripherals used by I2C
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);

	// Setup GPIO
	GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);
	GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);

	// Set GPIO D0 and D1 as SCL and SDA
	GPIOPinConfigure(GPIO_PA6_I2C1SCL);
	GPIOPinConfigure(GPIO_PA7_I2C1SDA);

	// Initialize as master - 'true' for fastmode, 'false' for regular
	if (fastMode){
		I2CMasterInitExpClk(I2C1_BASE, SysCtlClockGet(), true);
	}
	else{
		I2CMasterInitExpClk(I2C1_BASE, SysCtlClockGet(), false);
	}

}

void FloatToPrint(float floatValue, uint32_t splitValue[2]){
	int32_t i32IntegerPart;
	int32_t i32FractionPart;

        i32IntegerPart = (int32_t) floatValue;
        i32FractionPart = (int32_t) (floatValue * 1000.0f);
        i32FractionPart = i32FractionPart - (i32IntegerPart * 1000);
        if(i32FractionPart < 0)
        {
            i32FractionPart *= -1;
        }

	splitValue[0] = i32IntegerPart;
	splitValue[1] = i32FractionPart;
}




// Main ----------------------------------------------------------------------------------------------
int main(void){

	// Enable lazy stacking
	FPUEnable();
	FPULazyStackingEnable();

	// Set the system clock to run at 40Mhz off PLL with external crystal as reference.
	SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

	// Initialize the UART and write status.
	ConfigureUART();
	UARTprintf("BMP180 Example\n");

	// Enable LEDs
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, LED_RED|LED_BLUE|LED_GREEN);

	// Enable Timer
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

	ui32Period = (SysCtlClockGet()) / 2;
	TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period -1);

	IntEnable(INT_TIMER0A);
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	IntMasterEnable();

	TimerEnable(TIMER0_BASE, TIMER_A);

	// Enable I2C1
	ConfigureI2C1(true);

	
	BMP180Initialize(&BmpSensHub, 3);
	BMP180GetCalVals(&BmpSensHub, &BmpSensHubCals);

	// Malloc testing
	// tBMP180 *structTest = malloc(sizeof(tBMP180));
	// BMP180Initialize(structTest, 3);

	// get an initial sample started
	// may have to wait one conversion time before reading BMP180
	// OR toss the first reading
	BMP180GetRawTempStart();
	BMP180GetRawPressureStart(BmpSensHub.oversamplingSetting);

	while(1){

	}

}

// ISR - Every ~500ms call what is now in while loop
void BMP180ReadISR(void) {
	// Clear the timer interrupt
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	// Get & print temperature
	BMP180GetTemp(&BmpSensHub, &BmpSensHubCals);
	FloatToPrint(BmpSensHub.temp, printValue);
	UARTprintf("%d.%03d, ",printValue[0], printValue[1]);

	// Get & print pressure
	BMP180GetPressure(&BmpSensHub, &BmpSensHubCals);
	UARTprintf("%d\n", BmpSensHub.pressure);

	// initiate another sample to read next time around
	BMP180GetRawTempStart();
	BMP180GetRawPressureStart(BmpSensHub.oversamplingSetting);
}
