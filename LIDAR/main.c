#include <stdbool.h>
#include <stdint.h>
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"
#include "I2C_init.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "driverlib/interrupt.h"
#include "inc/hw_ints.h"
#include "inc/hw_types.h"
#include "driverlib/interrupt.h"


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

void main(void)
{
    // Set the clocking to run directly from the external crystal/oscillator.
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_INT | SYSCTL_XTAL_16MHZ);

    ConfigureUART();

    // 16 bit integer to store reading in.
    uint16_t Distance;

    // Initilize I2C protocoll
    InitI2C1();

    // Prepare Lidar for higher sample-rate, less accurate.
    I2CSend(LidarWriteAddr,0x00,0x04);

    // Infite while-loop
    int32_t count = 0;

    while(1)
    {
    	// Distance
    	Distance=ReadDistance();
    	if(count == 80000) {
    		UARTprintf('%d \n', Distance);
    		count=0;
    	}
    	count++;

    };
}
