/*
 * main.c
 *
 *  Created on: Apr 12, 2017
 *      Author: SeniorDesign
 */


//
// A boolean that is set when a BMP180 command has completed.
//
volatile bool g_bBMP180Done;
//
// The function that is provided by this example as a callback when BMP180
// transactions have completed.
//
void
BMP180Callback(void *pvCallbackData, uint_fast8_t ui8Status)
{
//
// See if an error occurred.
//
	if(ui8Status != I2CM_STATUS_SUCCESS)
	{
	//
	// An error occurred, so handle it here if required.
	//
	}
	//
	// Indicate that the BMP180 transaction has completed.
	//
	g_bBMP180Done = true;
}

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

//
// The BMP180 example.
//
void
BMP180Example(void)
{
	float fTemperature, fPressure;
	tI2CMInstance sI2CInst;
	tBMP180 sBMP180;
	//
	// Initialize the BMP180. This code assumes that the I2C master instance
	// has already been initialized.
	//
	g_bBMP180Done = false;
	BMP180Init(&sBMP180, &sI2CInst, 0x77, BMP180Callback, 0);
	while(!g_bBMP180Done)
	{
	}
	//
	// Configure the BMP180 for 2x oversampling.
	//
	g_bBMP180Done = false;
	BMP180ReadModifyWrite(&sBMP180, BMP180_O_CTRL_MEAS,~BMP180_CTRL_MEAS_OSS_M, BMP180_CTRL_MEAS_OSS_2,BMP180Callback,0);
	while(!g_bBMP180Done)
	{
	}
	//
	// Loop forever reading data from the BMP180. Typically, this process
	// would be done in the background, but for the purposes of this example,
	// it is shown in an infinite loop.
	//
	while(1)
	{
	//
	// Request another reading from the BMP180.
	//
	g_bBMP180Done = false;
	BMP180DataRead(&sBMP180, BMP180Callback, 0);
	while(!g_bBMP180Done)
	{
	}
	//
	// Get the new pressure and temperature reading.
	//
	BMP180DataPressureGetFloat(&sBMP180, &fPressure);
	BMP180DataTemperatureGetFloat(&sBMP180, &fTemperature);
	//
	// Do something with the new pressure and temperature readings.
	//
	}
}

