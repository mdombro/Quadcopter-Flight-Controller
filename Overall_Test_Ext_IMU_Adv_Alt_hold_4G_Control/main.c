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
#include <stdlib.h>
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
#include <math.h>
#include "sensorlib/i2cm_drv.h"
#include "driverlib/timer.h"
#include "ESC.h"
#include "PID.h"
#include "Adafruit_BNO055.h"
#include "PulsedLight_driver.h"
#include "bmpLib.h"


// Function prototypes
float Map_f(float in, float inMin, float inMax, float outMin, float outMax);

int32_t Map(int32_t in, int32_t inMin, int32_t inMax, int32_t outMin, int32_t outMax);

float Constrain(float input, float floor, float ceiling);

void RxCapture();

void initReceiver();

void IMUupdate();

float Abs(float in);

void ReadPulsedLight();

void PiUARTInt();

void BMP180ReadISR();

void CompFilter(float LPaltitude, float LPaltitudeBaro);

void PressuretoAlt(int32_t pressure);

//******************************************************************************************
//
//	Defines Section
//		- Clock period in us
// 		- Receiver Defines - watch clock_period
// 	 	- PID update rate
//		- Null zone size for RPY
//******************************************************************************************
// Converts timer clock ticks to us
// 40 for 40Mhz
#define CLOCK_PERIOD 1.0/40.0

#define AILERON_MAX 1890.0
#define AILERON_MIN 975.0
#define ELEVATOR_MAX 1928.0
#define ELEVATOR_MIN 991.0
#define THROTTLE_MAX 1947.0
#define THROTTLE_MIN 958.0
#define RUDDER_MAX 1966.0
#define RUDDER_MIN 1028.0
#define EMERGENCY_MAX 1909
#define EMERGENCY_MIN 1033
#define ANGLE_RATE_RANGE 250.0
#define ANGLE_RANGE 20.0
#define ANGLE_RUDDER_RATE_RANGE 220.0
#define THROTTLE_THRESHOLD 20

#define PID_UPDATE_FREQUENCY 50

#define AILERON_NULL_SIZE 4
#define ELEVATOR_NULL_SIZE 4
#define RUDDER_NULL_SIZE 20 //15

#define LPF_BETA 0.8
#define LPF_BETA_BAR 1.0
#define OFF_LPF_BETA 0.05

#define TAU 0.985

#define GAME_PAD_CONTROL 0
#define TX_CONTROL 1

//*****************************************************************************
//
// Global instance structure for the I2C master driver.
//
//*****************************************************************************
tI2CMInstance g_sI2CInst;


//*****************************************************************************
//
// Global instance structure for the IMU
//
//*****************************************************************************
Adafruit_BNO055 BNO;

//*****************************************************************************
//
// Global variables to hold orientation data
//
//*****************************************************************************
float Eulers[3];
float Gyros[3];

//*****************************************************************************
//
// Global variables to hold Receiver Values
//
//*****************************************************************************
volatile int32_t aileron, elevator, throttle, rudder, Switch, aileron_start, elevator_start, throttle_start, rudder_start, switch_start;
volatile uint8_t aile_on, ele_on, thro_on, rud_on, Switch_on;
volatile bool SWITCH;
volatile bool alt_start_flag;
volatile bool alt_start_1;


//*****************************************************************************
//
// Global struct for the PIDs and global output variables
// PID inputs will be gyro rates (pfGyro) and Eulers angles (pfEulers)
//
//*****************************************************************************
PID RateRoll;
PID RatePitch;
PID RateYaw;
PID AngleRoll;
PID AnglePitch;
PID AngleYaw;
PID AltHold;
float RateRollOutput, RatePitchOutput, RateYawOutput;
float aileron_f, elevator_f, rudder_f, throttle_f;
float aileronRate, elevatorRate, rudderRate;
float radians[3];
float yawTarget;
float initYawCount;
float rudderSamples[5];
float yawSample[10];
float rudderAverage;
float yawAverage;
float throttleAverage;
float hover_error;
float p_hover;
float AltHoldTarget;
float prevAltHoldTarget;
float AltHoldVelocity;
volatile float throttle_PID;
volatile float throttle_final;
volatile float hover_offset;

//*****************************************************************************
//
// Pulsed Light distance variable
// 		Distance given in cm
//
//*****************************************************************************
volatile int16_t PulsedLightDistance;
volatile float altitudeCal, LPaltitude;
volatile int8_t count_alt;
volatile bool alt_init_flag;
volatile bool ALT_prev;
volatile bool ALTHOLD;

//*****************************************************************************
//
// Arming State Variables
//
//*****************************************************************************
bool IS_ARMED;
bool ARM_state_poll;
uint32_t ARM_start;
volatile float altitudeInit;

//*****************************************************************************
//
// Xbox Controller vars
//
//*****************************************************************************
char RxBuffer[20];
uint8_t GLOBAL_SOURCE_STATE;
bool GLOBAL_SHUTDOWN;
bool GLOBAL_ARMED;
float GLOBAL_YAW, GLOBAL_THROTTLE, GLOBAL_ROLL, GLOBAL_PITCH;
uint16_t Pi_yaw, Pi_throttle, Pi_roll, Pi_pitch;
uint8_t state;
//char* bitties;

//*****************************************************************************
//
// BMP180 vars
//
//*****************************************************************************
tBMP180 BmpSensHub;
tBMP180Cals BmpSensHubCals;
volatile int8_t count_altBar;
volatile bool alt_initBar_flag;
volatile float altitudeBaro;
volatile float LPaltitudeBaro;
volatile float altitude;

//*****************************************************************************
//
// Raspberry PI UART Interrupt
//
//*****************************************************************************

//TIME OUT IF READINGS ARE CONSISTENT FOR X AMOUNT OF TIME -> SHOULD TRIGGER GPS LOCK TO NOT CAUSE CRASHES
void PiUARTInt() {
	TimerIntClear(TIMER5_BASE, TIMER_TIMA_TIMEOUT);
	IntPriorityMaskSet(0b01000000);
	UARTEchoSet(false);

	int i = 1;
	//uint8_t count;
	char bitties[256] = {};
	char terminationPos = (char) UARTPeek((char)0x0a);
	if (UARTRxBytesAvail() >= 22) {
		char checker = 'n';
		while(checker != (char) 0xaa) checker = UARTgetc();
		bitties[0] = checker;
		//while(UARTRxBytesAvail() > 0 ){
		for (i = 1; i < 11; i++) {
			//terminationPos = (char) UARTPeek('\r');
			bitties[i] = UARTgetc();//(char) UARTgets(bitties, (uint32_t) 256);
		}
	}

	if (i > 1) {
		//UARTgets(RxBuffer, 11);
		if (bitties[0] == 0xAA) {
			state = bitties[1];
			//if (!(state & 0b00000001)) GLOBAL_SOURCE_STATE = GAME_PAD_CONTROL;
			//else GLOBAL_SOURCE_STATE = GAME_PAD_CONTROL;
			if(state & 0b01000000) GLOBAL_ARMED = true;
			else GLOBAL_ARMED = false;
			if (state & 0b1000000) GLOBAL_SHUTDOWN = true;
			else GLOBAL_SHUTDOWN = false;

			//if (GLOBAL_SOURCE_STATE == GAME_PAD_CONTROL) {
			Pi_yaw = (bitties[2] << 8) | bitties[7];
			Pi_throttle = (bitties[4] << 8) | bitties[9];
			Pi_roll = (bitties[6] << 8) | bitties[3];
			Pi_pitch = (bitties[8] << 8) | bitties[5];

		}
	}
	IntPriorityMaskSet(0);
}

//*****************************************************************************
//
// Receiver interrupt function to capture pulse widths
//
//*****************************************************************************
void RxCapture() {
	//UARTprintf("%3d %3d %3d %3d %3d\n", aileron, elevator, throttle, rudder, EMERGENCY);
	//GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, 255);
	// Interrupt to handle receiver decoding
	// Each if statement will start timer counting on the rising edge (with some checks)
	// On falling edge for that pin timer is stopped and the value asessed
	// First channel is Switch stop - if its over a certain amount of time a global SYSTEM_OFF flag is set
	// Other channels are proportional channels
	// Will be scaled to whatever value is necessary for the application
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	IntPriorityMaskSet(0b00100000);
	if (GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_2) == GPIO_PIN_2) {
		GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_2);  // Clear interrupt flag
		// Aileron
		aile_on = 1;
		ele_on = 0;
		thro_on = 0;
		rud_on = 0;
		Switch_on = 0;
		aileron_start = TimerValueGet(TIMER0_BASE, TIMER_A);
	}
	else if (GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_3) == GPIO_PIN_3) {
		GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_3);  // Clear interrupt flag
		// elevator
		aile_on = 0;
		ele_on = 1;
		thro_on = 0;
		rud_on = 0;
		Switch_on = 0;
		elevator_start = TimerValueGet(TIMER0_BASE, TIMER_A);
	}
	else if (GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_1) == GPIO_PIN_1) {
		GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_1);  // Clear interrupt flag
		// throttle
		aile_on = 0;
		ele_on = 0;
		thro_on = 1;
		rud_on = 0;
		Switch_on = 0;
		throttle_start = TimerValueGet(TIMER0_BASE, TIMER_A);
	}
	else if (GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_4) == GPIO_PIN_4) {
		GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_4);  // Clear interrupt flag
		// rudder
		aile_on = 0;
		ele_on = 0;
		thro_on = 0;
		rud_on = 1;
		Switch_on = 0;
		rudder_start = TimerValueGet(TIMER0_BASE, TIMER_A);
	}
	else if (GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_0) == GPIO_PIN_0) {
		GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_0);  // Clear interrupt flag
		// Switch
		aile_on = 0;
		ele_on = 0;
		thro_on = 0;
		rud_on = 0;
		Switch_on = 1;
		switch_start = TimerValueGet(TIMER0_BASE, TIMER_A);
	}
	else {
		//TimerDisable(TIMER0_BASE, TIMER_A);
		if (aile_on && (GPIOIntStatus(GPIO_PORTE_BASE, false) & 0b100)) {
			GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_2);  // Clear interrupt flag
			aile_on = 0;
			ele_on = 0;
			thro_on = 0;
			rud_on = 0;
			Switch_on = 0;
			aileron = (TimerValueGet(TIMER0_BASE, TIMER_A) - aileron_start)*CLOCK_PERIOD;
			//aileron_f = Map_f((float)aileron, AILERON_MIN, AILERON_MAX, ANGLE_RANGE, -ANGLE_RANGE);
		}
		else if (ele_on && (GPIOIntStatus(GPIO_PORTE_BASE, false) & 0b1000)) {
			GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_3);  // Clear interrupt flag
			aile_on = 0;
			ele_on = 0;
			thro_on = 0;
			rud_on = 0;
			Switch_on = 0;
			elevator = (TimerValueGet(TIMER0_BASE, TIMER_A) - elevator_start)*CLOCK_PERIOD;
			//elevator_f = Map_f((float)elevator, ELEVATOR_MIN, ELEVATOR_MAX, -ANGLE_RANGE, ANGLE_RANGE);
		}
		else if (thro_on && (GPIOIntStatus(GPIO_PORTE_BASE, false) & 0b10)) {
			GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_1);  // Clear interrupt flag
			aile_on = 0;
			ele_on = 0;
			thro_on = 0;
			rud_on = 0;
			Switch_on = 0;
			throttle = (TimerValueGet(TIMER0_BASE, TIMER_A) - throttle_start)*CLOCK_PERIOD;
//			if(ALTHOLD && alt_start_flag) {
//				throttle_f = 0;
//				alt_start_flag = false;
//			}
//			else if(ALTHOLD && !alt_start_flag) {
//				throttle_f = Map_f((float)throttle, THROTTLE_MIN, THROTTLE_MAX, -45.0, 45.0);
//			}
//			else {
//				throttle_f = Map_f((float)throttle, THROTTLE_MIN, THROTTLE_MAX, 0.0, 90.0);
//			}
		}
		else if (rud_on && (GPIOIntStatus(GPIO_PORTE_BASE, false) & 0b10000)) {
			GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_4);  // Clear interrupt flag
			aile_on = 0;
			ele_on = 0;
			thro_on = 0;
			rud_on = 0;
			Switch_on = 0;
			rudder = (TimerValueGet(TIMER0_BASE, TIMER_A) - rudder_start)*CLOCK_PERIOD;
			//rudder_f = Map_f((float)rudder, RUDDER_MIN, RUDDER_MAX, ANGLE_RUDDER_RATE_RANGE, -ANGLE_RUDDER_RATE_RANGE);
		}
		else if (Switch_on && (GPIOIntStatus(GPIO_PORTE_BASE, false) & 0b01)) {
			GPIOIntClear(GPIO_PORTE_BASE, GPIO_PIN_0);  // Clear interrupt flag
			aile_on = 0;
			ele_on = 0;
			thro_on = 0;
			rud_on = 0;
			Switch_on = 0;
			Switch = (TimerValueGet(TIMER0_BASE, TIMER_A) - switch_start)*CLOCK_PERIOD;
//			if (althold > 1500) {
//				ALTHOLD = true;
//				if (!alt_start_1) {
//					alt_start_1 = true;
//					alt_start_flag = true;
//				}
//			}
//			else {
//				ALTHOLD = false;
//				alt_start_1 = false;
//			}
			//althold = Map(althold, EMERGENCY_MIN, EMERGENCY_MAX, 0, 10);
		}
		else {
			aile_on = 0;
			ele_on = 0;
			thro_on = 0;
			rud_on = 0;
			Switch_on = 0;
		}
	}
	//UARTprintf("%4d %4d\n", (int)throttle, EMERGENCY);
	//UARTprintf("%4d\n",EMERGENCY);
	//GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, 0);
	IntPriorityMaskSet(0);
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

	SWITCH	 = false;
}

//*****************************************************************************
//
// General purpose map function for receiver, PWM, and PID applications
//
//*****************************************************************************
int32_t Map(int32_t in, int32_t inMin, int32_t inMax, int32_t outMin, int32_t outMax) {
	return ((in - inMin)*(outMax - outMin) / (inMax - inMin)) + outMin;
}

float Map_f(float in, float inMin, float inMax, float outMin, float outMax) {
	return ((in - inMin)*(outMax - outMin) / (inMax - inMin)) + outMin;
}


//*****************************************************************************
//
// Configure the UART and its pins.
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
    UARTStdioConfig(0, 9600, 16000000);
}

//*****************************************************************************
//
// PID Computation and ESC Update
//
//*****************************************************************************
void PIDUpdate() {
	TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

	if (Switch > 1500) {
		GLOBAL_SOURCE_STATE = TX_CONTROL;
	}
	else {
		GLOBAL_SOURCE_STATE = GAME_PAD_CONTROL;
	}

	if (GLOBAL_SOURCE_STATE == GAME_PAD_CONTROL) {
		GLOBAL_YAW = Map_f( (float)(Pi_yaw), 0, 65535,  ANGLE_RUDDER_RATE_RANGE, -ANGLE_RUDDER_RATE_RANGE);
		if (ALTHOLD) {
			GLOBAL_THROTTLE = Map_f( (float)(Pi_throttle), 0, 65535, -45.0 , 45.0);
		}
		else if (!ALTHOLD) {
			GLOBAL_THROTTLE = Map_f( (float)(Pi_throttle), 0, 65535, 10.0, 90.0);
		}
		GLOBAL_ROLL = Map_f( (float)(Pi_roll), 0, 65535, -ANGLE_RANGE, ANGLE_RANGE);
		GLOBAL_PITCH = Map_f( (float)(Pi_pitch), 0, 65535, -ANGLE_RANGE, ANGLE_RANGE);
	}
	else if (GLOBAL_SOURCE_STATE == TX_CONTROL) {
		GLOBAL_YAW = Map_f((float)rudder, RUDDER_MIN, RUDDER_MAX, ANGLE_RUDDER_RATE_RANGE, -ANGLE_RUDDER_RATE_RANGE);
		GLOBAL_THROTTLE = Map_f((float)throttle, THROTTLE_MIN, THROTTLE_MAX, 0.0, 90.0);
		GLOBAL_ROLL = Map_f((float)aileron, AILERON_MIN, AILERON_MAX, ANGLE_RANGE, -ANGLE_RANGE);
		GLOBAL_PITCH = Map_f((float)elevator, ELEVATOR_MIN, ELEVATOR_MAX, -ANGLE_RANGE, ANGLE_RANGE);
	}

	//UARTprintf("%d %d %d %d\n", (int)GLOBAL_YAW, (int)GLOBAL_THROTTLE, (int)GLOBAL_ROLL, (int)GLOBAL_PITCH);
	if (GLOBAL_ARMED) {
		int i;
		for(i = 0; i < 5; i++) {
			if(i+1 < 5) {
				rudderSamples[i] = rudderSamples[i+1];
			}
			else {
				rudderSamples[i] = GLOBAL_YAW;
			}
		}


		int sum = 0;
		for(i = 0; i < 5; i++) {
				sum = sum+rudderSamples[i];
			}
		rudderAverage = sum/5.0;


		// Adding null zone for input channel roll, pitch, yaw
		if (GLOBAL_ROLL < AILERON_NULL_SIZE && GLOBAL_ROLL > -AILERON_NULL_SIZE) {
			GLOBAL_ROLL = 0;
		}
		if (GLOBAL_PITCH < ELEVATOR_NULL_SIZE && GLOBAL_PITCH > -ELEVATOR_NULL_SIZE) {
			GLOBAL_PITCH = 0;
		}

		if(ALTHOLD && ALT_prev == false) {
			SetMode(&AltHold, AUTOMATIC);
			ALT_prev = true;
		}
		else if(!ALTHOLD) {
			SetMode(&AltHold, MANUAL);
			throttle_PID = 0;
		}

		if (ALTHOLD && ALT_prev == true) {

			//THIS IS TESTING PURPOSE
			GLOBAL_THROTTLE = Map_f((float)(throttle), THROTTLE_MIN, THROTTLE_MAX, -45.0 , 45.0);
			if (GLOBAL_THROTTLE > 10.0) {
				AltHoldVelocity = GLOBAL_THROTTLE - 10.0;
			}
			else if (GLOBAL_THROTTLE < -10.0) {
				AltHoldVelocity = GLOBAL_THROTTLE + 10.0;
			}
			else {
				AltHoldVelocity = 0.0;
			}
			prevAltHoldTarget = AltHoldTarget;
			AltHoldTarget = AltHoldVelocity/50.0 + AltHoldTarget;

			Compute(&AltHold);
			//throttleAverage = throttleAverage-(OFF_LPF_BETA*(throttleAverage-throttle_PID));
			throttle_PID = throttle_PID + hover_offset;
			throttle_PID = Constrain(throttle_PID, 20.0, 90.0);
			throttle_final = throttle_PID;
//			if(AltHoldTarget == prevAltHoldTarget) {
//				hover_error = throttleAverage;
//				hover_offset = hover_offset + p_hover*hover_error;
//			}
		}
		else {
			AltHoldTarget = LPaltitude;
			throttle_final = GLOBAL_THROTTLE;
		}

		Compute(&AngleRoll);
		Compute(&AnglePitch);
//		Compute(&AngleYaw);
		if ((GLOBAL_YAW < RUDDER_NULL_SIZE && GLOBAL_YAW > -RUDDER_NULL_SIZE)) {
				rudderRate = 0;
		}
		else {
			rudderRate = GLOBAL_YAW;
			yawTarget = Eulers[0];
		}
		Compute(&RateRoll);
		Compute(&RatePitch);
		Compute(&RateYaw);
	//	PWM1 - Front Left
	//	PWM2 - Back left
	//	PWM3 - Front right
	//	PWM4 - Back right
		writePWM1(Constrain(throttle_final - RateRollOutput - RatePitchOutput + RateYawOutput, 15, 100));  // Front left
		writePWM2(Constrain(throttle_final - RateRollOutput + RatePitchOutput - RateYawOutput, 15, 100));
		writePWM3(Constrain(throttle_final + RateRollOutput - RatePitchOutput - RateYawOutput, 15, 100));
		writePWM4(Constrain(throttle_final + RateRollOutput + RatePitchOutput + RateYawOutput, 15, 100));
	}
	else {
		writePWM1(0);  // Front left
		writePWM2(0);
		writePWM3(0);
		writePWM4(0);
	}
}

float Abs(float in) {
	if (in < 0.0) {
		return -1.0*in;
	}
	else {
		return in;
	}
}

//*****************************************************************************
//
// IMU Interrupt
//
//*****************************************************************************
void IMUupdate() {
	TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
	IntPriorityMaskSet(0b01100000);
	if(initYawCount > 0) {
		initYawCount--;
		yawTarget = Eulers[0];
	}
	getGyros(&BNO, Gyros);
	getEulers(&BNO, Eulers);

//*****************************************************************************
//
// DO NOT FOR THE LOVE OF GOD I MEAN FOR THE LOVE OF GOD CHANGE THIS
//
//*****************************************************************************
	Gyros[1] = -Gyros[1];
//*****************************************************************************
//
// THE ABOVE LINE IS SACRED...IT RUNS THE UNIVERSE
//
//*****************************************************************************

//	Gyros[0] -= 0.0097;
//	Gyros[1] -= -0.0084;
//	Gyros[2] -= 0.0099;
//	Eulers[1] -= -0.9375;
//	Eulers[2] -= 0.6875;
	IntPriorityMaskSet(0);
	//UARTprintf("%4d,%4d,%4d,%4d,%4d\n", (int)(Eulers[1]*1e7), (int)(Eulers[2]*1e7), (int)(Gyros[0]*1e7), (int)(Gyros[1]*1e7), (int)(Gyros[2]*1e7));
	//UARTprintf("%4d %4d %4d\n", (int)(aileron_f*1e3), (int)(elevator_f*1e3), (int)(GLOBAL_ROLL*1e3));
}

//*****************************************************************************
//
// LIDAR Interrupt
//
//*****************************************************************************
void ReadPulsedLight() {
	TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
	PulsedLightDistance = ReadDistance();
	float roll = Eulers[2]*(M_PI/180.0);
	float pitch = Eulers[1]*(M_PI/180.0);
	roll = roll < 0.0 ? -1.0*roll : roll;
	pitch = pitch < 0.0 ? -1.0*pitch : pitch;
	float x = sin(roll);
	float y = sin(pitch);
	float z = sqrt(1.0 - pow(x,2) - pow(y,2));
	float l = sqrt(pow(x,2) + pow(y,2));
	float theta = atan2(z,l);
	//theta = (M_PI/2.0) - theta;
	altitudeCal = sin(theta)*(float)PulsedLightDistance;
	LPaltitude = LPaltitude-(LPF_BETA*(LPaltitude-altitudeCal));
	if (count_alt < 0) {
		alt_init_flag = true;
	}
	count_alt--;
	UARTprintf("Fuck You\n");
	UARTprintf("$%d %d;", (int)(LPaltitude), (int)AltHoldTarget);
	//UARTprintf("%4d\n",PulsedLightDistance);
}


//*****************************************************************************
//
// BMP180 Interrupt
//
//*****************************************************************************
void BMP180ReadISR(void) {
	// Clear the timer interrupt
	TimerIntClear(TIMER1_BASE, TIMER_TIMB_TIMEOUT);
	// Get & print temperature
	BMP180GetTemp(&BmpSensHub, &BmpSensHubCals);
	//FloatToPrint(BmpSensHub.temp, printValue);
	//UARTprintf("%d.%03d, ",printValue[0], printValue[1]);

	// Get & print pressure
	BMP180GetPressure(&BmpSensHub, &BmpSensHubCals);
	//pressure = BmpSensHub.pressure;
	//UARTprintf("%d\n", BmpSensHub.pressure);
	PressuretoAlt(BmpSensHub.pressure);
	LPaltitudeBaro = LPaltitudeBaro-(LPF_BETA_BAR*(LPaltitudeBaro-altitudeBaro));

	CompFilter(LPaltitude,LPaltitudeBaro);

	// initiate another sample to read next time around
	BMP180GetRawTempStart();
	BMP180GetRawPressureStart(BmpSensHub.oversamplingSetting);
	if (count_altBar < 0) {
		alt_initBar_flag = true;
	}
	count_altBar--;
}

//*****************************************************************************
//
// Pressure to Altitude Conversion
//
//*****************************************************************************
void PressuretoAlt(int32_t pressure) {
	//GOTTA FIGURE OUT THE UNITS OF PRESSURE TO DETERMINE IF A GAIN FACOTR IS NEEDED
	altitudeBaro = (1-pow(((float)pressure/1013.25),0.190284))*145366.46;
}

//*****************************************************************************
//
// LIDAR & BMP180 Complimentary Filter
//
//*****************************************************************************
void CompFilter(float LPaltitude, float LPaltitudeBaro) {
	if(altitude <= 100) {
		altitude = (TAU)*LPaltitude+(1-TAU)*LPaltitudeBaro;
	}
	else if(altitude > 100) {
		altitude = (1-TAU)*LPaltitude+(TAU)*LPaltitudeBaro;
	}
}

//*****************************************************************************
//
// Constrain values to a given range
//
//*****************************************************************************
float Constrain(float input, float floor, float ceiling) {
	if (input > ceiling)
		return ceiling;
	else if (input < floor) {
		return floor;
	}
	else
		return input;
}


//*****************************************************************************
//
// Main application entry point.
//
//*****************************************************************************
int
main(void)
{

	//**********************************************************
	//
	// Initial Variable Assignment
	//
	//**********************************************************
 	aile_on = 0;
	ele_on = 0;
	thro_on = 0;
	rud_on = 0;
	Switch_on = 0;
	aileron = 0;
	elevator = 0;
	throttle = 0;
	throttle_f = 0.0;
	rudder = 0;
	ALTHOLD = true;
	alt_start_flag = false;
	alt_start_1 = false;
	LPaltitude = 0;
	LPaltitudeBaro = 0;
	throttle_final = 0;
	throttleAverage = 0.0;
	hover_error = 0;
	p_hover = 0.01;
	hover_offset = 54;
	rudderSamples[0] = rudderSamples[1] = rudderSamples[2] = rudderSamples[3] = rudderSamples[4] = 0;
	ALT_prev = false;

	GLOBAL_THROTTLE = 0;
	GLOBAL_YAW = 0;
	GLOBAL_ROLL = 0;
	GLOBAL_PITCH = 0;

	IS_ARMED = false;
	ARM_state_poll = false;

	GLOBAL_ARMED = false;

	FPUEnable();
	//bitties = malloc( sizeof(char) * 256 );


    //**********************************************************
	//
	// System Clock Set Up
	//
	//**********************************************************
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);


    //**********************************************************
	//
	// UART Configuration
	//
	//**********************************************************
    ConfigureUART();

    //**********************************************************
    //
    // PWM/PID Interrupt Timer Configuration
    //
    //**********************************************************
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
	TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
	uint32_t ui32Period = SysCtlClockGet() / PID_UPDATE_FREQUENCY;
	TimerLoadSet(TIMER1_BASE, TIMER_A, ui32Period -1);

	IntPrioritySet(INT_TIMER1A, 0b01100000);
	IntEnable(INT_TIMER1A);
	TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

	//**********************************************************
	//
	// BNO055 Initialization
	//
	//**********************************************************
	Adafruit_BNO055_init(&BNO, -1, BNO055_ADDRESS_A);
	if(!begin(&BNO, OPERATION_MODE_NDOF)) {
		UARTprintf("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
		while(1);
	}
	delay(100);
	setExtCrystalUse(&BNO, true);
	setSensorOffsets(&BNO);
	setMode(&BNO, OPERATION_MODE_NDOF);

	//**********************************************************
	//
	// IMU Interrupt Timer Configuration
	//
	//**********************************************************
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
	TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);
	uint32_t ui32Period2 = SysCtlClockGet() / 100;
	TimerLoadSet(TIMER2_BASE, TIMER_A, ui32Period2 - 1);

	IntPrioritySet(INT_TIMER2A, 0b01000000);
	IntEnable(INT_TIMER2A);
	TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
	IntMasterEnable();


	//**********************************************************
	//
	// PWM Configuration
	//
	//**********************************************************
    ConfigurePWM();

    IntMasterEnable();

    //**********************************************************
	//
	// PID Configuration
	//
	//**********************************************************
    //aileronRate
    //elevatorRate
    //rudderRate
    PID_Make(&RateRoll, &Gyros[0], &RateRollOutput, &aileronRate, 0.075, 0.001, 0.045, DIRECT);
    PID_Make(&RatePitch, &Gyros[1], &RatePitchOutput, &elevatorRate, 0.075, 0.001, 0.045, DIRECT);
    PID_Make(&RateYaw, &Gyros[2], &RateYawOutput, &rudderRate, 0.3, 0.0, 0.0, DIRECT);
    SetMode(&RateRoll, AUTOMATIC);
    SetMode(&RatePitch, AUTOMATIC);
    SetMode(&RateYaw, AUTOMATIC);
    SetOutputLimits(&RateRoll, -50, 50);
    SetOutputLimits(&RatePitch, -50, 50);
    SetOutputLimits(&RateYaw, -40, 40);

    PID_Make(&AngleRoll, &Eulers[2], &aileronRate, &GLOBAL_ROLL, 4.5, 0.05, 0, DIRECT);
    PID_Make(&AnglePitch, &Eulers[1], &elevatorRate, &GLOBAL_PITCH, 4.5, 0.05, 0, DIRECT);
    PID_Make(&AngleYaw, &Eulers[0], &rudderRate, &yawTarget, 0.2, 0, 0.3, DIRECT);
    SetMode(&AngleRoll, AUTOMATIC);
	SetMode(&AnglePitch, AUTOMATIC);
	SetMode(&AngleYaw, AUTOMATIC);
	SetOutputLimits(&AngleRoll, -ANGLE_RATE_RANGE, ANGLE_RATE_RANGE);
	SetOutputLimits(&AnglePitch, -ANGLE_RATE_RANGE, ANGLE_RATE_RANGE);
	SetOutputLimits(&AngleYaw, -ANGLE_RUDDER_RATE_RANGE, ANGLE_RUDDER_RATE_RANGE);

	PID_Make(&AltHold, &LPaltitude, &throttle_PID, &AltHoldTarget, 0.3, 0.0001, 1.0, DIRECT);
	SetMode(&AltHold, MANUAL);
	SetOutputLimits(&AltHold, -20.0, 20.0);

	SysCtlDelay(6000000); //80000000   800000

	//**********************************************************
	//
	// Pulsed Light LIDAR Initialization
	//
	//**********************************************************
	PulsedLightInit();
	PulsedLightDistance = 0;

	//**********************************************************
	//
	// BMP180 Pressure Initialization
	//
	//**********************************************************

	//Dunno if the FPUEnable() and FPULazyStackingEnable() are necessary...
//	FPULazyStackingEnable();
//	BMP180Initialize(&BmpSensHub,3);
//	BMP180GetCalVals(&BmpSensHub, &BmpSensHubCals);
//	BMP180GetRawTempStart();
//	BMP180GetRawPressureStart(BmpSensHub.oversamplingSetting);

	//**********************************************************
	//
	// Pulsed Light Interrupt Timer Configuration
	//
	//**********************************************************
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
	TimerConfigure(TIMER3_BASE, TIMER_CFG_PERIODIC);
	uint32_t ui32Period3 = SysCtlClockGet() / 50;
	TimerLoadSet(TIMER3_BASE, TIMER_A, ui32Period3 - 1);

	IntPrioritySet(INT_TIMER3A, 0b11100000);
	IntEnable(INT_TIMER3A);
	TimerIntEnable(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
	IntMasterEnable();

	//**********************************************************
	//
	// BMP180 Barometer Timer Interrupt Configuration
	//
	//**********************************************************
//	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
//	TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
//	uint32_t ui32Period1B = (SysCtlClockGet()) / 50;
//	TimerLoadSet(TIMER1_BASE, TIMER_B, ui32Period1B -1);
//
//	IntPrioritySet(INT_TIMER1B, 0b11000000);
//	IntEnable(INT_TIMER1B);
//	TimerIntEnable(TIMER1_BASE, TIMER_TIMB_TIMEOUT);
//	IntMasterEnable();
//
//	// Enable I2C
//	ConfigureI2C1(true);

	//**********************************************************
	//
	// Yaw Initialization
	//
	//**********************************************************
	yawTarget = 0;
	initYawCount = 100;

	//**********************************************************
	//
	// Initialize PulsedLight readings and altitude target
	//
	//**********************************************************
	TimerEnable(TIMER3_BASE, TIMER_A);

	count_alt = 20;
	alt_init_flag = false;

	while (!alt_init_flag) {
		AltHoldTarget = LPaltitude;
	}
	altitudeInit = LPaltitude;

	//**********************************************************
	//
	//Initialize BMP180 readings and reference measurement
	//
	//**********************************************************
//	TimerEnable(TIMER1_BASE, TIMER_B);
//
//	count_altBar = 20;
//	alt_initBar_flag = false;
//
//	while(!alt_initBar_flag) {
//		AltHoldTarget = altitude;
//	}

	//**********************************************************
	//
	// Initialize Receiver readings
	//
	//**********************************************************
	initReceiver();

	//**********************************************************
	//
	// Initialize timer interrupts for PIDs and IMU
	//
	//**********************************************************
	TimerEnable(TIMER1_BASE, TIMER_A);
	TimerEnable(TIMER2_BASE, TIMER_A);

	//**********************************************************
	//
	// Pi UART Initialization
	//
	//**********************************************************
//	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5);
//	TimerConfigure(TIMER5_BASE, TIMER_CFG_PERIODIC);
//	uint32_t ui32Period5 = SysCtlClockGet() / 100;
//	TimerLoadSet(TIMER5_BASE, TIMER_A, ui32Period5 - 1);
//
//	IntPrioritySet(INT_TIMER5A, 0b00100000);
//	//IntRegister(INT_TIMER3A, UARTInt);
//	TimerIntRegister(TIMER5_BASE, TIMER_A, PiUARTInt);
//	IntEnable(INT_TIMER5A);
//	TimerIntEnable(TIMER5_BASE, TIMER_TIMA_TIMEOUT);
//
//	TimerEnable(TIMER5_BASE, TIMER_A);
//	UARTEchoSet(false);

	//**********************************************************
	//
	// Arm state poll timer
	//
	//**********************************************************
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER4);
	TimerConfigure(TIMER4_BASE, TIMER_CFG_PERIODIC);
	//TimerLoadSet(TIMER4_BASE, TIMER_A, );
	TimerEnable(TIMER4_BASE, TIMER_A);

//	IntPrioritySet(INT_TIMER3A, 0b11100000);
//	IntEnable(INT_TIMER3A);
//	TimerIntEnable(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
//	IntMasterEnable();

    IntMasterEnable();

    uint32_t count = 0;
    while(1)
    {
    	// No throttle and rudder right - arming sequence

    	if (throttle < THROTTLE_MIN + 110 && rudder > RUDDER_MAX - 100 && !GLOBAL_ARMED) {
    		//TimerLoadSet(TIMER4_BASE, TIMER_A, 200000);
    		//TimerEnable(TIMER4_BASE, TIMER_A);
    		ARM_start = TimerValueGet(TIMER4_BASE, TIMER_A);
    		while (ARM_start - TimerValueGet(TIMER4_BASE, TIMER_A) < 40000000) {
    			if (throttle < THROTTLE_MIN + 110 && rudder > RUDDER_MAX - 100) ARM_state_poll = true;
    			else {ARM_state_poll = false; break;}
    		}
    		if (ARM_state_poll) GLOBAL_ARMED = true;
    		//TimerDisable(TIMER4_BASE, TIMER_A);
    	}
    	// No throttle and rudder left - disarming sequence
    	else if (throttle < THROTTLE_MIN + 110 && rudder < RUDDER_MIN + 100 && GLOBAL_ARMED && LPaltitude <= altitudeInit+100) {
    		//TimerLoadSet(TIMER4_BASE, TIMER_A, 200000);
			//TimerEnable(TIMER4_BASE, TIMER_A);
    		ARM_start = TimerValueGet(TIMER4_BASE, TIMER_A);
			while (ARM_start - TimerValueGet(TIMER4_BASE, TIMER_A) < 10000000) {
				if (throttle < THROTTLE_MIN + 110 && rudder < RUDDER_MIN + 100) ARM_state_poll = true;
				else {ARM_state_poll = false; break;}
			}
			if (ARM_state_poll) GLOBAL_ARMED = false;
			//TimerDisable(TIMER4_BASE, TIMER_A);
    	}

//    	count++;
//    	if (count == 30000) {
//    		count = 0;
//    		UARTprintf("%d %d %d\n", IS_ARMED, throttle, rudder);
//    	}

	}
}
