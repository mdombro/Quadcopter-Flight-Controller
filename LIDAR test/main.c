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

#include "I2C_init.h"


uint16_t LidarWriteAddr = 0xc4;
uint16_t LidarReadAddr = 0xc5;
uint16_t Lidar2ByteRead = 0x8f;


int16_t ReadDistance()
{
	// Integer to store data
	uint16_t Dist = 0;

	// Prepare Lidar for reading
	I2CSend(LidarWriteAddr,0x00,0x04);

	//wait for 20 ms
	SysCtlDelay(SysCtlClockGet() / (160));

	// Read Data
	Dist =  I2CReceive(LidarReadAddr,LidarWriteAddr,Lidar2ByteRead);

    return Dist;
}

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




// Main ----------------------------------------------------------------------------------------------
int main(void){
	// Set the clocking to run directly from the external crystal/oscillator.
	//SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_INT | SYSCTL_XTAL_16MHZ);
	SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

	ConfigureUART();
	UARTprintf("LIDAR beam test\n");

	// 16 bit integer to store reading in.
	uint16_t Distance;

	// Initilize I2C protocol
	InitI2C1();

	// Prepare Lidar for higher sample-rate, less accurate.
	I2CSend(LidarWriteAddr,0x00,0x04);

	I2CSend(LidarWriteAddr,0x13,0b11111100);

	// Infite while-loop
	int32_t count = 0;

	while(1)
	{
		// Distance
		Distance=ReadDistance();
		if(count == 5) {
			UARTprintf("%d\n", Distance);
			count=0;
		}
		count++;

	}
}
