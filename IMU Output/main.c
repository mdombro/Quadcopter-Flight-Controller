#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "sensorlib/hw_mpu9150.h"
#include "sensorlib/hw_ak8975.h"
#include "sensorlib/i2cm_drv.h"
#include "sensorlib/ak8975.h"
#include "sensorlib/mpu9150.h"
#include "sensorlib/comp_dcm.h"
#include "driverlib/fpu.h"
#include "driverlib/debug.h"

//*****************************************************************************
//
// Define MPU9150 I2C Address.
//
//*****************************************************************************
#define MPU9150_I2C_ADDRESS     0x68

//*****************************************************************************
//
// Define MPU9150 data sampling frequency.
//
//*****************************************************************************
#define MOTION_SAMPLE_FREQ_HZ   50

//*****************************************************************************
//
// Weights the DCM should use for each sensor.  Must add to 1.0
//
//*****************************************************************************
#define DCM_MAG_WEIGHT          0.2f
#define DCM_GYRO_WEIGHT         0.6f
#define DCM_ACCEL_WEIGHT        0.2f

//*****************************************************************************
//
// Global instance structure for the I2C master driver.
//
//*****************************************************************************
tI2CMInstance g_sI2CInst;

//*****************************************************************************
//
// Global instance structure for the MPU9150 sensor driver.
//
//*****************************************************************************
tMPU9150 g_sMPU9150Inst;

//*****************************************************************************
//
// Global Instance structure to manage the DCM state.
//
//*****************************************************************************
tCompDCM g_sCompDCMInst;

//*****************************************************************************
//
// Global storage for most recent Euler angles and Sensor Data
//
//*****************************************************************************
float g_pfEulers[3];
float g_pfAccel[3];
float g_pfGyro[3];
float g_pfMag[3];

//*****************************************************************************
//
// Global flags to alert main that MPU9150 I2C transaction error has occurred.
//
//*****************************************************************************
volatile uint_fast8_t g_vui8ErrorFlag;

//*****************************************************************************
//
// Global system tick counter holds elapsed time since the application started
// expressed in 100ths of a second.
//
//*****************************************************************************
volatile uint_fast32_t g_ui32SysTickCount;

//*****************************************************************************
//
// Called by the NVIC as a result of I2C3 Interrupt. I2C3 is the I2C connection
// to the MPU9150.
//
//*****************************************************************************
void
MotionI2CIntHandler(void)
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
// MPU9150 Sensor callback function.  Called at the end of MPU9150 sensor
// driver transactions. This is called from I2C interrupt context.
//
//*****************************************************************************
void MotionCallback(void* pvCallbackData, uint_fast8_t ui8Status)
{
    //
    // If the transaction succeeded set the data flag to indicate to
    // application that this transaction is complete and data may be ready.
    //
    if(ui8Status == I2CM_STATUS_SUCCESS)
    {
        //
        // Set the motion event flag to show that we have completed the
        // i2c transfer
        //
//        HWREGBITW(&g_ui32Events, MOTION_EVENT) = 1;

        //
        // Turn on the LED to show we are ready to process motion date
        //
//        g_pui32RGBColors[RED] = 0xFFFF;
//        RGBColorSet(g_pui32RGBColors);

//        if(g_ui8MotionState == MOTION_STATE_RUN);
//        {
            //
            // Get local copies of the raw motion sensor data.
            //
            MPU9150DataAccelGetFloat(&g_sMPU9150Inst, g_pfAccel, g_pfAccel + 1,
                                     g_pfAccel + 2);

            MPU9150DataGyroGetFloat(&g_sMPU9150Inst, g_pfGyro, g_pfGyro + 1,
                                    g_pfGyro + 2);

            MPU9150DataMagnetoGetFloat(&g_sMPU9150Inst, g_pfMag, g_pfMag + 1,
                                       g_pfMag + 2);

            //
            // Update the DCM. Do this in the ISR so that timing between the
            // calls is consistent and accurate.
            //
            CompDCMMagnetoUpdate(&g_sCompDCMInst, g_pfMag[0], g_pfMag[1],
                                 g_pfMag[2]);
            CompDCMAccelUpdate(&g_sCompDCMInst, g_pfAccel[0], g_pfAccel[1],
                               g_pfAccel[2]);
            CompDCMGyroUpdate(&g_sCompDCMInst, -g_pfGyro[0], -g_pfGyro[1],
                              -g_pfGyro[2]);
            CompDCMUpdate(&g_sCompDCMInst);
//        }
    }
    else
    {
        //
        // An Error occurred in the I2C transaction.
        //
//        HWREGBITW(&g_ui32Events, MOTION_ERROR_EVENT) = 1;
//        g_ui8MotionState = MOTION_STATE_ERROR;
//        g_ui32RGBMotionBlinkCounter = g_ui32SysTickCount;
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
    uint32_t ui32Status;

    ui32Status = GPIOIntStatus(GPIO_PORTB_BASE, true);

    //
    // Clear all the pin interrupts that are set
    //
    GPIOIntClear(GPIO_PORTB_BASE, ui32Status);

    //
    // Check which GPIO caused the interrupt event.
    //
    if(ui32Status & GPIO_PIN_2)
    {
        //
        // The MPU9150 data ready pin was asserted so start an I2C transfer
        // to go get the latest data from the device.
        //
        MPU9150DataRead(&g_sMPU9150Inst, MotionCallback, &g_sMPU9150Inst);
    }
}

//*****************************************************************************
//
// MPU9150 Application error handler.
//
//*****************************************************************************
void
MotionErrorHandler(char * pcFilename, uint_fast32_t ui32Line)
{
    //
    // Set terminal color to red and print error status and locations
    //
    UARTprintf("\033[31;1m");
    UARTprintf("Error: %d, File: %s, Line: %d\nSee I2C status definitions in "
               "utils\\i2cm_drv.h\n", g_vui8ErrorFlag, pcFilename, ui32Line);

    //
    // Return terminal color to normal
    //
    UARTprintf("\033[0m");
}

//*****************************************************************************
//
// Function to wait for the MPU9150 transactions to complete.
//
//*****************************************************************************
void
MotionI2CWait(char* pcFilename, uint_fast32_t ui32Line)
{
    //
    // Put the processor to sleep while we wait for the I2C driver to
    // indicate that the transaction is complete.
    //
    while( (g_vui8ErrorFlag == 0) )
    {
        //
        // Do Nothing
        //
    }

    //
    // clear the event flag.
    //
    //HWREGBITW(&g_ui32Events, MOTION_EVENT) = 0;

    //
    // If an error occurred call the error handler immediately.
    //
    if(g_vui8ErrorFlag)
    {
        MotionErrorHandler(pcFilename, ui32Line);
        g_vui8ErrorFlag = 0;
    }

    return;
}

void
MotionInit(void)
{
    //
    // Enable port B used for motion interrupt.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    //
    // The I2C3 peripheral must be enabled before use.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C3);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    //
    // Configure the pin muxing for I2C3 functions on port D0 and D1.
    //
    GPIOPinConfigure(GPIO_PD0_I2C3SCL);
    GPIOPinConfigure(GPIO_PD1_I2C3SDA);

    //
    // Select the I2C function for these pins.  This function will also
    // configure the GPIO pins pins for I2C operation, setting them to
    // open-drain operation with weak pull-ups.  Consult the data sheet
    // to see which functions are allocated per pin.
    //
    GPIOPinTypeI2CSCL(GPIO_PORTD_BASE, GPIO_PIN_0);
    GPIOPinTypeI2C(GPIO_PORTD_BASE, GPIO_PIN_1);

    //
    // Configure and Enable the GPIO interrupt. Used for INT signal from the
    // MPU9150
    //
    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOIntEnable(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_FALLING_EDGE);
    IntEnable(INT_GPIOB);

    //
    // Enable interrupts to the processor.
    //
    IntMasterEnable();

    //
    // Initialize I2C3 peripheral.
    //
    I2CMInit(&g_sI2CInst, I2C3_BASE, INT_I2C3, 0xff, 0xff,
             SysCtlClockGet());

    //
    // Set the motion state to initializing.
    //
//    g_ui8MotionState = MOTION_STATE_INIT;

    //
    // Initialize the MPU9150 Driver.
    //
    MPU9150Init(&g_sMPU9150Inst, &g_sI2CInst, MPU9150_I2C_ADDRESS,
                MotionCallback, &g_sMPU9150Inst);

    //
    // Wait for transaction to complete
    //
    MotionI2CWait(__FILE__, __LINE__);

    //
    // Write application specifice sensor configuration such as filter settings
    // and sensor range settings.
    //
    g_sMPU9150Inst.pui8Data[0] = MPU9150_CONFIG_DLPF_CFG_44_42;
    g_sMPU9150Inst.pui8Data[1] = MPU9150_GYRO_CONFIG_FS_SEL_2000;
    g_sMPU9150Inst.pui8Data[2] = (MPU9150_ACCEL_CONFIG_ACCEL_HPF_2_5HZ |
                                  MPU9150_ACCEL_CONFIG_AFS_SEL_4G);
    MPU9150Write(&g_sMPU9150Inst, MPU9150_O_CONFIG, g_sMPU9150Inst.pui8Data, 3,
                 MotionCallback, &g_sMPU9150Inst);

    //
    // Wait for transaction to complete
    //
    MotionI2CWait(__FILE__, __LINE__);

    //
    // Configure the data ready interrupt pin output of the MPU9150.
    //
    g_sMPU9150Inst.pui8Data[0] = (MPU9150_INT_PIN_CFG_INT_LEVEL |
                                  MPU9150_INT_PIN_CFG_INT_RD_CLEAR |
                                  MPU9150_INT_PIN_CFG_LATCH_INT_EN);
    g_sMPU9150Inst.pui8Data[1] = MPU9150_INT_ENABLE_DATA_RDY_EN;
    MPU9150Write(&g_sMPU9150Inst, MPU9150_O_INT_PIN_CFG,
                 g_sMPU9150Inst.pui8Data, 2, MotionCallback, &g_sMPU9150Inst);

    //
    // Wait for transaction to complete
    //
    MotionI2CWait(__FILE__, __LINE__);

    //
    // Initialize the DCM system.
    //
    CompDCMInit(&g_sCompDCMInst, 1.0f / ((float) MOTION_SAMPLE_FREQ_HZ),
                DCM_ACCEL_WEIGHT, DCM_GYRO_WEIGHT, DCM_MAG_WEIGHT);
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
    UARTStdioConfig(0, 115200, 16000000);
}

void
SysTickIntHandler(void)
{
    g_ui32SysTickCount++;
//    HWREGBITW(&g_ui32Events, USB_TICK_EVENT) = 1;
//    HWREGBITW(&g_ui32Events, LPRF_TICK_EVENT) = 1;
//    g_ui8Buttons = ButtonsPoll(0, 0);
}

int main(void) {
	SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
	 //
	// Turn on stacking of FPU registers if FPU is used in the ISR.
	//
	FPULazyStackingEnable();
	 //
	// Set the system tick to fire 100 times per second.
	//
	SysTickPeriodSet(SysCtlClockGet() / 100);  // SYSTICKS_PER_SECOND
	SysTickIntEnable();
	SysTickEnable();
	//
	// Enable the Debug UART.
	//
	ConfigureUART();
	//
	// Print the welcome message to the terminal.
	//
	UARTprintf("\033[2JAir Mouse Application\n");
	//
	// Configure desired interrupt priorities. This makes certain that the DCM
	// is fed data at a consistent rate. Lower numbers equal higher priority.
	//
	IntPrioritySet(INT_I2C3, 0x00);
	IntPrioritySet(INT_GPIOB, 0x10);
	IntPrioritySet(FAULT_SYSTICK, 0x20);
	IntPrioritySet(INT_UART1, 0x60);
	IntPrioritySet(INT_UART0, 0x70);
	IntPrioritySet(INT_WTIMER5B, 0x80);

	 //
	// Initialize the motion sub system.
	//
	MotionInit();

	if(g_sMPU9150Inst.pui8Data[14] & AK8975_ST1_DRDY) {
		//
		// Get local copy of Accel and Mag data to feed to the DCM
		// start.
		//
		MPU9150DataAccelGetFloat(&g_sMPU9150Inst, g_pfAccel,
								 g_pfAccel + 1, g_pfAccel + 2);
		MPU9150DataMagnetoGetFloat(&g_sMPU9150Inst, g_pfMag,
								  g_pfMag + 1, g_pfMag + 2);
		MPU9150DataGyroGetFloat(&g_sMPU9150Inst, g_pfGyro,
								g_pfGyro + 1, g_pfGyro + 2);

		//
		// Feed the initial measurements to the DCM and start it.
		// Due to the structure of our MotionMagCallback function,
		// the floating point magneto data is already in the local
		// data buffer.
		//
		CompDCMMagnetoUpdate(&g_sCompDCMInst, g_pfMag[0], g_pfMag[1],
							 g_pfMag[2]);
		CompDCMAccelUpdate(&g_sCompDCMInst, g_pfAccel[0], g_pfAccel[1],
						   g_pfAccel[2]);
		CompDCMStart(&g_sCompDCMInst);

		//
		// Proceed to the run state.
		//
		//g_ui8MotionState = MOTION_STATE_RUN;
	}

	while (1) {
		//
		// Get the latest Euler data from the DCM. DCMUpdate is done
		// inside the interrupt routine to insure it is not skipped and
		// that the timing is consistent.
		//
		CompDCMComputeEulers(&g_sCompDCMInst, g_pfEulers,
							 g_pfEulers + 1, g_pfEulers + 2);
		UARTprintf("\033[2J%f  %f  $f", g_pfEulers, g_pfEulers + 1, g_pfEulers + 2);
	}
	
}
