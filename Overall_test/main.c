//*****************************************************************************
//
// compdcm_mpu9150.c - Example use of the SensorLib with the MPU9150
//
// Copyright (c) 2013-2016 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.3.156 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************

#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288419716939937510582
#endif

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
#include "drivers/rgb.h"  // careful with this one, try to find another way to drive LEDs
#include "sensorlib/hw_mpu9150.h"
#include "sensorlib/hw_ak8975.h"
#include "sensorlib/i2cm_drv.h"
#include "sensorlib/ak8975.h"
#include "sensorlib/mpu9150.h"
#include "driverlib/timer.h"
#include "Orientation_filter.h"
#include "ESC.h"

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>Nine Axis Sensor Fusion with the MPU9150 and Complimentary-Filtered
//! DCM (compdcm_mpu9150)</h1>
//!
//! This example demonstrates the basic use of the Sensor Library, TM4C123G
//! LaunchPad and SensHub BoosterPack to obtain nine axis motion measurements
//! from the MPU9150.  The example fuses the nine axis measurements into a set
//! of Euler angles: roll, pitch and yaw.  It also produces the rotation
//! quaternions.  The fusion mechanism demonstrated is complimentary-filtered
//! direct cosine matrix (DCM) algorithm is provided as part of the Sensor
//! Library.
//!
//! Connect a serial terminal program to the LaunchPad's ICDI virtual serial
//! port at 115,200 baud.  Use eight bits per byte, no parity and one stop bit.
//! The raw sensor measurements, Euler angles and quaternions are printed to
//! the terminal.  The RGB LED begins to blink at 1Hz after initialization is
//! completed and the example application is running.
//
//*****************************************************************************

//*****************************************************************************
//
// Define MPU9150 I2C Address.
//
//*****************************************************************************
#define MPU9150_I2C_ADDRESS     0x68

//*****************************************************************************
//
// Base update period for the system
//
//*****************************************************************************
#define DELTA_T 0.002


//******************************************************************************************
//
//   Receiver Defines - watch clock_period
//
//******************************************************************************************
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


//*****************************************************************************
//
// Global array for holding the color values for the RGB.
//
//*****************************************************************************
uint32_t g_pui32Colors[3];

//*****************************************************************************
//
// Global instance structure for the I2C master driver.
//
//*****************************************************************************
tI2CMInstance g_sI2CInst;

//*****************************************************************************
//
// Global instance structure for the ISL29023 sensor driver.
//
//*****************************************************************************
tMPU9150 g_sMPU9150Inst;


//*****************************************************************************
//
// Global flags to alert main that MPU9150 I2C transaction is complete
//
//*****************************************************************************
volatile uint_fast8_t g_vui8I2CDoneFlag;

//*****************************************************************************
//
// Global flags to alert main that MPU9150 I2C transaction error has occurred.
//
//*****************************************************************************
volatile uint_fast8_t g_vui8ErrorFlag;

//*****************************************************************************
//
// Global flags to alert main that MPU9150 data is ready to be retrieved.
//
//*****************************************************************************
volatile uint_fast8_t g_vui8DataFlag;

//*****************************************************************************
//
// Global counter to control and slow down the rate of data to the terminal.
//
//*****************************************************************************
#define PRINT_SKIP_COUNT        1

uint32_t g_ui32PrintSkipCounter;

//*****************************************************************************
//
// Global variables to hold orientation data
//
//*****************************************************************************
//int_fast32_t i32IPart[16], i32FPart[16];
uint_fast32_t ui32Idx, ui32CompDCMStarted;
float pfData[16];
float *pfAccel, *pfGyro, *pfMag, *pfEulers, *pfQuaternion;
uint16_t count = 0;

//*****************************************************************************
//
// Global variables to hold Receiver Values
//
//*****************************************************************************
volatile uint32_t aileron, elevator, throttle, rudder, emergency, aileron_start, elevator_start, throttle_start, rudder_start, emergency_start;
volatile uint8_t aile_on, ele_on, thro_on, rud_on, emerg_on;
volatile bool EMERGENCY;

//*****************************************************************************
//
// Initialization counter to average out Accel and mag based initial orientation
//
//*****************************************************************************
uint16_t initCount;

//*****************************************************************************
//
// Global construct for Orientation Filter state
//
//*****************************************************************************
OFilter Filter;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//*****************************************************************************
//
// MPU9150 Sensor callback function.  Called at the end of MPU9150 sensor
// driver transactions. This is called from I2C interrupt context. Therefore,
// we just set a flag and let main do the bulk of the computations and display.
//
//*****************************************************************************
void
MPU9150AppCallback(void *pvCallbackData, uint_fast8_t ui8Status)
{
    //
    // If the transaction succeeded set the data flag to indicate to
    // application that this transaction is complete and data may be ready.
    //
    if(ui8Status == I2CM_STATUS_SUCCESS)
    {
        g_vui8I2CDoneFlag = 1;
    }

    //
    // Store the most recent status in case it was an error condition
    //
    g_vui8ErrorFlag = ui8Status;
}

//*****************************************************************************
//
// Called by the NVIC as a result of GPIO port B interrupt event. For this
// application GPIO port B pin 2 is the interrupt line for the MPU9150
//
//*****************************************************************************
void
IntGPIOb(void)
{
    unsigned long ulStatus;
    float quat[4];   // This will go away eventually

    ulStatus = GPIOIntStatus(GPIO_PORTB_BASE, true);

    //
    // Clear all the pin interrupts that are set
    //
    GPIOIntClear(GPIO_PORTB_BASE, ulStatus);

    if(ulStatus & GPIO_PIN_2)
    {
        //
        // MPU9150 Data is ready for retrieval and processing.
        //
        MPU9150DataRead(&g_sMPU9150Inst, MPU9150AppCallback, &g_sMPU9150Inst);
    }

    //
	// Get floating point version of the Accel Data in m/s^2.
	//
	MPU9150DataAccelGetFloat(&g_sMPU9150Inst, pfAccel, pfAccel + 1,
							pfAccel + 2);

	//
	// Get floating point version of angular velocities in rad/sec
	//
	MPU9150DataGyroGetFloat(&g_sMPU9150Inst, pfGyro, pfGyro + 1,
						   pfGyro + 2);

	//
	// Get floating point version of magnetic fields strength in tesla
	//
	MPU9150DataMagnetoGetFloat(&g_sMPU9150Inst, pfMag, pfMag + 1,
							  pfMag + 2);

	// RE-orient Gyro readings
	pfGyro[0] = -pfGyro[0];
	pfGyro[1] = -pfGyro[1];

	// For 50 iterations average out the estimated orientation given accel and mag data
	if (initCount > 0) {
		initCount--;
		Orientation_Filter_Update(&Filter, pfGyro, pfAccel, 1);
		//SLERP(&Filter, Filter.qCurrent, Filter.qAccelMag, Filter.qCurrent, GAIN_SLERP_AVERAGING);
		getQuaternion(&Filter, quat);
	}

	Orientation_Filter_Update(&Filter, pfGyro, pfAccel, 0);
	//getEuler(&Filter, pfEulers);

//	if (count%10 == 0) {
//		getQuaternion(&Filter, quat);
//		UARTprintf("%d,%d,%d,%d\n", (int)(quat[0]*1e7), (int)(quat[1]*1e7), (int)(quat[2]*1e7), (int)(quat[3]*1e7));
//	}
//	count++;
}

//*****************************************************************************
//
// Called by the NVIC as a result of I2C3 Interrupt. I2C3 is the I2C connection
// to the MPU9150.
//
//*****************************************************************************
void
MPU9150I2CIntHandler(void)
{
    //
    // Pass through to the I2CM interrupt handler provided by sensor library.
    // This is required to be at application level so that I2CMIntHandler can
    // receive the instance structure pointer as an argument.
    //
    I2CMIntHandler(&g_sI2CInst);
}


//*****************************************************************************
//
// MPU9150 Application error handler. Show the user if we have encountered an
// I2C error.
//
//*****************************************************************************
void
MPU9150AppErrorHandler(char *pcFilename, uint_fast32_t ui32Line)
{
    //
    // Set terminal color to red and print error status and locations
    //
    UARTprintf("\033[31;1m");
    UARTprintf("Error: %d, File: %s, Line: %d\n"
               "See I2C status definitions in sensorlib\\i2cm_drv.h\n",
               g_vui8ErrorFlag, pcFilename, ui32Line);

    //
    // Return terminal color to normal
    //
    UARTprintf("\033[0m");

    //
    // Set RGB Color to RED
    //
    g_pui32Colors[0] = 0xFFFF;
    g_pui32Colors[1] = 0;
    g_pui32Colors[2] = 0;
    RGBColorSet(g_pui32Colors);

    //
    // Increase blink rate to get attention
    //
    RGBBlinkRateSet(10.0f);

    //
    // Go to sleep wait for interventions.  A more robust application could
    // attempt corrective actions here.
    //
    while(1)
    {
        //
        // Do Nothing
        //
    }
}


//*****************************************************************************
//
// Function to wait for the MPU9150 transactions to complete. Use this to spin
// wait on the I2C bus.
//
//*****************************************************************************
void
MPU9150AppI2CWait(char *pcFilename, uint_fast32_t ui32Line)
{
    //
    // Put the processor to sleep while we wait for the I2C driver to
    // indicate that the transaction is complete.
    //
    while((g_vui8I2CDoneFlag == 0) && (g_vui8ErrorFlag == 0))
    {
        //
        // Do Nothing
        //
    }

    //
    // If an error occurred call the error handler immediately.
    //
    if(g_vui8ErrorFlag)
    {
        MPU9150AppErrorHandler(pcFilename, ui32Line);
    }

    //
    // clear the data flag for next use.
    //
    g_vui8I2CDoneFlag = 0;
}

//*****************************************************************************
//
// Receiver interrupt function to capture pulse widths
//
//*****************************************************************************
void RxCapture() {
	IntMasterDisable();
	//UARTprintf("%3d %3d %3d %3d %3d\n", aileron, elevator, throttle, rudder, EMERGENCY);
	//GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, 255);
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
		if (aile_on && (GPIOIntStatus(GPIO_PORTE_BASE, false) & 0b100)) {
			GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_2);  // Clear interrupt flag
			aile_on = 0;
			ele_on = 0;
			thro_on = 0;
			rud_on = 0;
			emerg_on = 0;
			aileron = (TimerValueGet(TIMER0_BASE, TIMER_A) - aileron_start)*CLOCK_PERIOD;
			aileron = Map(aileron, AILERON_MIN, AILERON_MAX, -50, 50);
		}
		else if (ele_on && (GPIOIntStatus(GPIO_PORTE_BASE, false) & 0b1000)) {
			GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_3);  // Clear interrupt flag
			aile_on = 0;
			ele_on = 0;
			thro_on = 0;
			rud_on = 0;
			emerg_on = 0;
			elevator = (TimerValueGet(TIMER0_BASE, TIMER_A) - elevator_start)*CLOCK_PERIOD;
			elevator = Map(elevator, ELEVATOR_MIN, ELEVATOR_MAX, -50, 50);
		}
		else if (thro_on && (GPIOIntStatus(GPIO_PORTE_BASE, false) & 0b10000)) {
			GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_4);  // Clear interrupt flag
			aile_on = 0;
			ele_on = 0;
			thro_on = 0;
			rud_on = 0;
			emerg_on = 0;
			throttle = (TimerValueGet(TIMER0_BASE, TIMER_A) - throttle_start)*CLOCK_PERIOD;
			throttle = Map(throttle, THROTTLE_MIN, THROTTLE_MAX, 0, 100);
		}
		else if (rud_on && (GPIOIntStatus(GPIO_PORTE_BASE, false) & 0b01)) {
			GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_0);  // Clear interrupt flag
			aile_on = 0;
			ele_on = 0;
			thro_on = 0;
			rud_on = 0;
			emerg_on = 0;
			rudder = (TimerValueGet(TIMER0_BASE, TIMER_A) - rudder_start)*CLOCK_PERIOD;
			rudder = Map(rudder, RUDDER_MIN, RUDDER_MAX, -50, 50);
		}
		else if (emerg_on && (GPIOIntStatus(GPIO_PORTE_BASE, false) & 0b10)) {
			GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_1);  // Clear interrupt flag
			aile_on = 0;
			ele_on = 0;
			thro_on = 0;
			rud_on = 0;
			emerg_on = 0;
			emergency = (TimerValueGet(TIMER0_BASE, TIMER_A) - emergency_start)*CLOCK_PERIOD;
			if (emergency > 1500) EMERGENCY = true;
			else EMERGENCY = false;
			//emergency = Map(emergency, EMERGENCY_MIN, EMERGENCY_MAX, 0, 10);
		}
		else {
			aile_on = 0;
			ele_on = 0;
			thro_on = 0;
			rud_on = 0;
			emerg_on = 0;
		}
	}
	//GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, 0);
	IntMasterEnable();
}

//*****************************************************************************
//
// Configure the receiver read interrupt pins and the timer to capture pulse width
//
//*****************************************************************************
void initReceiver() {
	// GPIO Port A setup
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	SysCtlDelay(3);
	GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_INT_PIN_2 | GPIO_INT_PIN_3 | GPIO_INT_PIN_4 | GPIO_INT_PIN_0 | GPIO_INT_PIN_1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlDelay(3);
	GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_5);

	//ConfigureUART();

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

	IntPrioritySet(INT_GPIOE, 0);
	IntEnable(INT_GPIOE);
	IntMasterEnable();

	EMERGENCY = false;
}

//*****************************************************************************
//
// General purpose map function for receiver, PWM, and PID applications
//
//*****************************************************************************
int32_t Map(int32_t in, int32_t inMin, int32_t inMax, int32_t outMin, int32_t outMax) {
	return ((in - inMin)*(outMax - outMin) / (inMax - inMin)) + outMin;
}


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
    UARTStdioConfig(0, 921600, 16000000);
}

//*****************************************************************************
//
// PID compute and ESC update
//
//*****************************************************************************
void PIDUpdate() {
	TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
	if (!EMERGENCY) {
		writePWM1(throttle);
		writePWM2(throttle);
		writePWM3(throttle);
		writePWM4(throttle);
	}
	else {
		writePWM1(0);
		writePWM2(0);
		writePWM3(0);
		writePWM4(0);
	}
}


//*****************************************************************************
//
// Main application entry point.
//
//*****************************************************************************
int
main(void)
{
	//
	// Initialize convenience pointers that clean up and clarify the code
	// meaning. We want all the data in a single contiguous array so that
	// we can make our pretty printing easier later.
	//
    pfAccel = pfData;
    pfGyro = pfData + 3;
    pfMag = pfData + 6;
    pfEulers = pfData + 9;
    pfQuaternion = pfData + 12;
    initCount = 50;

    //
    // Setup the system clock to run at 40 Mhz from PLL with crystal reference
    //
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    //
    // Enable port B used for motion interrupt.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_3);

    //
    // Initialize the UART.
    //
    ConfigureUART();


    //
    // Print the welcome message to the terminal.
    //
    UARTprintf("\033[2JMPU9150 Raw Example\n");

    //
    // The I2C3 peripheral must be enabled before use.
    //

    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C3);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    //
    // Configure the pin muxing for I2C3 functions on port D0 and D1.
    //
    ROM_GPIOPinConfigure(GPIO_PD0_I2C3SCL);
    ROM_GPIOPinConfigure(GPIO_PD1_I2C3SDA);

    //
    // Select the I2C function for these pins.  This function will also
    // configure the GPIO pins pins for I2C operation, setting them to
    // open-drain operation with weak pull-ups.  Consult the data sheet
    // to see which functions are allocated per pin.
    //
    GPIOPinTypeI2CSCL(GPIO_PORTD_BASE, GPIO_PIN_0);
    ROM_GPIOPinTypeI2C(GPIO_PORTD_BASE, GPIO_PIN_1);

    //
    // Configure and Enable the GPIO interrupt. Used for INT signal from the
    // MPU9150
    //
    ROM_GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOIntEnable(GPIO_PORTB_BASE, GPIO_PIN_2);
    ROM_GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_FALLING_EDGE);
    IntPrioritySet(INT_GPIOB, 0b01000000);
    ROM_IntEnable(INT_GPIOB);

    //
    // Enable interrupts to the processor.
    //
    ROM_IntMasterEnable();

    //
    // Initialize I2C3 peripheral.
    //
    I2CMInit(&g_sI2CInst, I2C3_BASE, INT_I2C3, 0xff, 0xff,
             ROM_SysCtlClockGet());

    //
    // Initialize the MPU9150 Driver.
    //
    MPU9150Init(&g_sMPU9150Inst, &g_sI2CInst, MPU9150_I2C_ADDRESS,
                MPU9150AppCallback, &g_sMPU9150Inst);

    //
    // Wait for transaction to complete
    //
    MPU9150AppI2CWait(__FILE__, __LINE__);

    //
    // Write sample rate into SMPLRT_DIV register
    //
    g_sMPU9150Inst.pui8Data[0] = 0x01;
    MPU9150Write(&g_sMPU9150Inst, MPU9150_O_SMPLRT_DIV, g_sMPU9150Inst.pui8Data, 1,
                     MPU9150AppCallback, &g_sMPU9150Inst);

    //
    // Wait for transaction to complete
	//
    MPU9150AppI2CWait(__FILE__, __LINE__);

    //
    // Write application specifice sensor configuration such as filter settings
    // and sensor range settings.
    //
    g_sMPU9150Inst.pui8Data[0] = MPU9150_CONFIG_DLPF_CFG_44_42;
    g_sMPU9150Inst.pui8Data[1] = MPU9150_GYRO_CONFIG_FS_SEL_250;
    g_sMPU9150Inst.pui8Data[2] = (MPU9150_ACCEL_CONFIG_ACCEL_HPF_2_5HZ |
                                  MPU9150_ACCEL_CONFIG_AFS_SEL_2G);
    MPU9150Write(&g_sMPU9150Inst, MPU9150_O_CONFIG, g_sMPU9150Inst.pui8Data, 3,
                 MPU9150AppCallback, &g_sMPU9150Inst);

    //
    // Wait for transaction to complete
    //
    MPU9150AppI2CWait(__FILE__, __LINE__);

    //
    // Configure the data ready interrupt pin output of the MPU9150.
    //
    g_sMPU9150Inst.pui8Data[0] = MPU9150_INT_PIN_CFG_INT_LEVEL |
                                    MPU9150_INT_PIN_CFG_INT_RD_CLEAR |
                                    MPU9150_INT_PIN_CFG_LATCH_INT_EN;
    g_sMPU9150Inst.pui8Data[1] = MPU9150_INT_ENABLE_DATA_RDY_EN;
    MPU9150Write(&g_sMPU9150Inst, MPU9150_O_INT_PIN_CFG,
                 g_sMPU9150Inst.pui8Data, 2, MPU9150AppCallback,
                 &g_sMPU9150Inst);

    //
    // Wait for transaction to complete
    //
    MPU9150AppI2CWait(__FILE__, __LINE__);

    Orientation_Filter_Init(&Filter, DELTA_T);

    // Get receiver interrupt monitoring going
    // Uses Port A oins for interrupt and Timer0 in full width mode
    IntMasterDisable();
    initReceiver();

    //**********************************************************
    // PWM/PID Interrupt Timer Config
    //**********************************************************
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
	TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
	uint32_t ui32Period = SysCtlClockGet() / 200;
	TimerLoadSet(TIMER1_BASE, TIMER_A, ui32Period -1);

	IntPrioritySet(INT_TIMER1A, 0b00100000);
	IntEnable(INT_TIMER1A);
	TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
	IntMasterEnable();

	TimerEnable(TIMER1_BASE, TIMER_A);

    ConfigurePWM();

    SysCtlDelay(800000); //80000000
    IntMasterEnable();

    aile_on = 0;
	ele_on = 0;
	thro_on = 0;
	rud_on = 0;
	emerg_on = 0;
	aileron = 0;
	elevator = 0;
	throttle = 0;
	rudder = 0;
	EMERGENCY = false;

    uint32_t count = 0;
    while(1)
    {
    	count++;
    	if (count == 50000) {
    		count = 0;
    		//IntMasterDisable();
    		UARTprintf("%d\n", throttle);
    		//IntMasterEnable();
    	}

	}
}



