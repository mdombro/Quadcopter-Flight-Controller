// bmpLib.c
//
//****************************************************************************************************
// Author:
// 	Nipun Gunawardena
//
// Credits:
//	Borrows from Texas Instruments' bmp180 library.
//
// Requirements:
// 	Requires Texas Instruments' TivaWare.
//
// Description:
// 	Interface with Bosch BMP180
//
// Notes:
//	See bmpLib.h
//	Designed to be used with I2C1
//	
//****************************************************************************************************


// Includes ------------------------------------------------------------------------------------------
#include <stdbool.h>
#include <stdint.h>
#include "driverlib/sysctl.h"
#include "driverlib/i2c.h"
#include "driverlib/rom.h"
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "driverlib/pin_map.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "bmpLib.h"

#define TARGET_IS_BLIZZARD_RB1


// Functions -----------------------------------------------------------------------------------------

void BMP180Initialize(tBMP180 *psInst, uint8_t oss){
	if(oss > 3){
		oss = 3;	
	}
	psInst->oversamplingSetting = oss;

	psInst->calRegs[0] = BMP180_REG_CAL_AC1;
	psInst->calRegs[1] = BMP180_REG_CAL_AC2;
	psInst->calRegs[2] = BMP180_REG_CAL_AC3;
	psInst->calRegs[3] = BMP180_REG_CAL_AC4;
	psInst->calRegs[4] = BMP180_REG_CAL_AC5;
	psInst->calRegs[5] = BMP180_REG_CAL_AC6;
	psInst->calRegs[6] = BMP180_REG_CAL_B1;
	psInst->calRegs[7] = BMP180_REG_CAL_B2;
	psInst->calRegs[8] = BMP180_REG_CAL_MB;
	psInst->calRegs[9] = BMP180_REG_CAL_MC;
	psInst->calRegs[10] = BMP180_REG_CAL_MD;

}


void BMP180GetCalVals(tBMP180 *psInst, tBMP180Cals *calInst){
	int i;
	for(i = 0; i < 11; i++){
		// Configure to write, set register to send, send
		I2CMasterSlaveAddrSet(I2C1_BASE, BMP180_I2C_ADDRESS, false);
		I2CMasterDataPut(I2C1_BASE, psInst->calRegs[i]);
		I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);

		// Wait for bus to free
		while(I2CMasterBusy(I2C1_BASE)){}

		// Send restart
		I2CMasterSlaveAddrSet(I2C1_BASE, BMP180_I2C_ADDRESS, true);

		// Read MSB
		I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
		while(I2CMasterBusy(I2C1_BASE)){}
		psInst->calRawVals[2*i] = I2CMasterDataGet(I2C1_BASE);
		
		// Read LSB
		I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
		while(I2CMasterBusy(I2C1_BASE)){}
		psInst->calRawVals[2*i+1] = I2CMasterDataGet(I2C1_BASE);
	}
	calInst->ac1 = (int16_t) ( (psInst->calRawVals[0] << 8) | psInst->calRawVals[1] );
	calInst->ac2 = (int16_t) ( (psInst->calRawVals[2] << 8) | psInst->calRawVals[3] );
	calInst->ac3 = (int16_t) ( (psInst->calRawVals[4] << 8) | psInst->calRawVals[5] );
	calInst->ac4 = (uint16_t)( (psInst->calRawVals[6] << 8) | psInst->calRawVals[7] );
	calInst->ac5 = (uint16_t)( (psInst->calRawVals[8] << 8) | psInst->calRawVals[9] );
	calInst->ac6 = (uint16_t)( (psInst->calRawVals[10] << 8) | psInst->calRawVals[11] );
	calInst->b1 =  (int16_t) ( (psInst->calRawVals[12] << 8) | psInst->calRawVals[13] );
	calInst->b2 =  (int16_t) ( (psInst->calRawVals[14] << 8) | psInst->calRawVals[15] );
	calInst->mb =  (int16_t) ( (psInst->calRawVals[16] << 8) | psInst->calRawVals[17] );
	calInst->mc =  (int16_t) ( (psInst->calRawVals[18] << 8) | psInst->calRawVals[19] );
	calInst->md =  (int16_t) ( (psInst->calRawVals[20] << 8) | psInst->calRawVals[21] );
	
}


void BMP180GetRawTempStart(){
	// Configure to write, send control register
	I2CMasterSlaveAddrSet(I2C1_BASE, BMP180_I2C_ADDRESS, false);
	I2CMasterDataPut(I2C1_BASE, BMP180_REG_CONTROL);
	I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);

	// Wait for bus to free
	while(I2CMasterBusy(I2C1_BASE)){}

	// Send temperature command
	I2CMasterDataPut(I2C1_BASE, BMP180_READ_TEMP);
	I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);

	// Wait for bus to free
	while(I2CMasterBusy(I2C1_BASE)){}
}

extern void BMP180GetRawTemp(tBMP180 *psInst) {
	// Configure to write, send tempdata register
	I2CMasterSlaveAddrSet(I2C1_BASE, BMP180_I2C_ADDRESS, false);
	I2CMasterDataPut(I2C1_BASE, BMP180_REG_TEMPDATA);
	I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);

	// Wait for bus to free
	while(I2CMasterBusy(I2C1_BASE)){}

	// Send restart
	I2CMasterSlaveAddrSet(I2C1_BASE, BMP180_I2C_ADDRESS, true);

	// Read MSB
	I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
	while(I2CMasterBusy(I2C1_BASE)){}
	psInst->tempRawVals[0] = I2CMasterDataGet(I2C1_BASE);

	// Read LSB
	I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
	while(I2CMasterBusy(I2C1_BASE)){}
	psInst->tempRawVals[1] = I2CMasterDataGet(I2C1_BASE);

}

//void BMP180GetRawTemp(tBMP180 *psInst){
//	// Configure to write, send control register
//	I2CMasterSlaveAddrSet(I2C1_BASE, BMP180_I2C_ADDRESS, false);
//	I2CMasterDataPut(I2C1_BASE, BMP180_REG_CONTROL);
//	I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
//
//	// Wait for bus to free
//	while(I2CMasterBusy(I2C1_BASE)){}
//
//	// Send temperature command
//	I2CMasterDataPut(I2C1_BASE, BMP180_READ_TEMP);
//	I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
//
//	// Wait for bus to free
//	while(I2CMasterBusy(I2C1_BASE)){}
//
//	// Delay for 4.5 ms
//	SysCtlDelay(SysCtlClockGet()/3/222);
//
//	// Configure to write, send tempdata register
//	I2CMasterSlaveAddrSet(I2C1_BASE, BMP180_I2C_ADDRESS, false);
//	I2CMasterDataPut(I2C1_BASE, BMP180_REG_TEMPDATA);
//	I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
//
//	// Wait for bus to free
//	while(I2CMasterBusy(I2C1_BASE)){}
//
//	// Send restart
//	I2CMasterSlaveAddrSet(I2C1_BASE, BMP180_I2C_ADDRESS, true);
//
//	// Read MSB
//	I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
//	while(I2CMasterBusy(I2C1_BASE)){}
//	psInst->tempRawVals[0] = I2CMasterDataGet(I2C1_BASE);
//
//	// Read LSB
//	I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
//	while(I2CMasterBusy(I2C1_BASE)){}
//	psInst->tempRawVals[1] = I2CMasterDataGet(I2C1_BASE);
//
//}

void BMP180GetTemp(tBMP180 *psInst, tBMP180Cals *calInst){
	// Get raw temperature
	BMP180GetRawTemp(psInst);

	// Calculate UT
	int32_t UT = (int32_t)((psInst->tempRawVals[0]<<8) + psInst->tempRawVals[1]);

	// Calculate X1
	int32_t X1 = (UT - (int32_t)calInst->ac6) * ((int32_t)calInst->ac5) / 32768;

	// Calculate X2
	int32_t X2 = ((int32_t)calInst->mc * 2048) / (X1 + (int32_t)calInst->md);

	// Calculate B5
	int32_t B5 = X1 + X2;

	psInst->temp = ( ((float)B5 + 8.0f)/16.0f )/10.0f;	// Divide by 10 because temp is in 0.1C, see datasheet

}

void BMP180GetRawPressureStart(int oss){
	// Configure to write, send control register
	I2CMasterSlaveAddrSet(I2C1_BASE, BMP180_I2C_ADDRESS, false);
	I2CMasterDataPut(I2C1_BASE, BMP180_REG_CONTROL);
	I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);

	// Wait for bus to free
	while(I2CMasterBusy(I2C1_BASE)){}

	// Send pressure command
	I2CMasterDataPut(I2C1_BASE, BMP180_READ_PRES_BASE + (oss << 6));
	I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);

	// Wait for bus to free
	while(I2CMasterBusy(I2C1_BASE)){}
}


//void BMP180GetRawPressure(tBMP180 *psInst, int oss){
//	// Configure to write, send control register
//	I2CMasterSlaveAddrSet(I2C1_BASE, BMP180_I2C_ADDRESS, false);
//	I2CMasterDataPut(I2C1_BASE, BMP180_REG_CONTROL);
//	I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
//
//	// Wait for bus to free
//	while(I2CMasterBusy(I2C1_BASE)){}
//
//	// Send pressure command
//	I2CMasterDataPut(I2C1_BASE, BMP180_READ_PRES_BASE + (oss << 6));
//	I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
//
//	// Wait for bus to free
//	while(I2CMasterBusy(I2C1_BASE)){}
//
//	// Delay based on oversampling setting
//	switch(oss){
//		case 0:
//			// Ultra low power - 4.5ms
//			SysCtlDelay(SysCtlClockGet()/3/222);
//			break;
//		case 1:
//			// Standard - 7.5 ms
//			SysCtlDelay(SysCtlClockGet()/3/133);
//			break;
//		case 2:
//			// High resolution - 13.5 ms
//			SysCtlDelay(SysCtlClockGet()/3/76);
//			break;
//		case 3:
//			// Ultra high resolution - 25.5 ms
//			SysCtlDelay(SysCtlClockGet()/3/39);
//			break;
//		default:
//			break;
//	}
//
//	// Configure to write, send presdata register
//	I2CMasterSlaveAddrSet(I2C1_BASE, BMP180_I2C_ADDRESS, false);
//	I2CMasterDataPut(I2C1_BASE, BMP180_REG_PRESSUREDATA);
//	I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
//
//	// Wait for bus to free
//	while(I2CMasterBusy(I2C1_BASE)){}
//
//	// Send restart
//	I2CMasterSlaveAddrSet(I2C1_BASE, BMP180_I2C_ADDRESS, true);
//
//	// Read MSB
//	I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
//	while(I2CMasterBusy(I2C1_BASE)){}
//	psInst->presRawVals[0] = I2CMasterDataGet(I2C1_BASE);
//
//	// Read LSB
//	I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
//	while(I2CMasterBusy(I2C1_BASE)){}
//	psInst->presRawVals[1] = I2CMasterDataGet(I2C1_BASE);
//
//	// Read XLSB
//	I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
//	while(I2CMasterBusy(I2C1_BASE)){}
//	psInst->presRawVals[2] = I2CMasterDataGet(I2C1_BASE);
//}

extern void BMP180GetRawPressure(tBMP180 *psInst) {
	// Configure to write, send presdata register
	I2CMasterSlaveAddrSet(I2C1_BASE, BMP180_I2C_ADDRESS, false);
	I2CMasterDataPut(I2C1_BASE, BMP180_REG_PRESSUREDATA);
	I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);

	// Wait for bus to free
	while(I2CMasterBusy(I2C1_BASE)){}

	// Send restart
	I2CMasterSlaveAddrSet(I2C1_BASE, BMP180_I2C_ADDRESS, true);

	// Read MSB
	I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
	while(I2CMasterBusy(I2C1_BASE)){}
	psInst->presRawVals[0] = I2CMasterDataGet(I2C1_BASE);

	// Read LSB
	I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
	while(I2CMasterBusy(I2C1_BASE)){}
	psInst->presRawVals[1] = I2CMasterDataGet(I2C1_BASE);

	// Read XLSB
	I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
	while(I2CMasterBusy(I2C1_BASE)){}
	psInst->presRawVals[2] = I2CMasterDataGet(I2C1_BASE);
}


void BMP180GetPressure(tBMP180 *psInst, tBMP180Cals *calInst){
	// Get raw temp, pressure
	BMP180GetRawTemp(psInst);
	BMP180GetRawPressure(psInst);
	
	// Calculate UT
	int32_t UT = (int32_t)((psInst->tempRawVals[0]<<8) + psInst->tempRawVals[1]);

	// Calculate UP
	int32_t UP = ( ((int32_t)psInst->presRawVals[0] << 16) + ((int32_t)psInst->presRawVals[1] << 8) + (int32_t)psInst->presRawVals[2]) >> (8 - psInst->oversamplingSetting);

	// Calculate X1
	int32_t X1 = (UT - (int32_t)calInst->ac6) * ((int32_t)calInst->ac5) / 32768;

	// Calculate X2
	int32_t X2 = ((int32_t)calInst->mc * 2048) / (X1 + (int32_t)calInst->md);

	// Calculate B5
	int32_t B5 = X1 + X2;

	// Calculate B6
	int32_t B6 = B5 - 4000;

	// Recalculate X1
	X1 = ((int32_t)calInst->b2 * ((B6 * B6) / 4096)) / 2048;

	// Recalculate X2
	X2 = (int32_t)calInst->ac2 * B6 / 2048;

	// Calculate X3
	int32_t X3 = X1 + X2;

	// Calculate B3
	int32_t B3 = ( ( ((int32_t)calInst->ac1*4 + X3) << psInst->oversamplingSetting) + 2 ) / 4;

	// Recalculate X1
	X1 = (int32_t)calInst->ac3 * B6 / 8192;

	// Recalculate X2
	X2 = ((int32_t)calInst->b1 * ((B6*B6) / 4096)) / 65536;

	// Recalculate X3
	X3 = ((X1 + X2) + 2) / 4;

	// Calculate B4
	uint32_t B4 = (uint32_t)calInst->ac4 * (uint32_t)(X3 + 32768) / 32768;

	// Calculate B7
	uint32_t B7 = ((uint32_t)UP - B3)*(50000 >> psInst->oversamplingSetting);

	// Calculate p
	int32_t p;
	if (B7 < 0x80000000){
		p = (B7 * 2) / B4;
	} else{
		p = (B7 / B4) * 2;
	}

	// Recalculate X1
	X1 = (p / 256) * (p / 256);
	X1 = (X1 * 3038) / 65536;

	// Recalculate X2
	X2 = (-7357 * p) / 65536;

	// Recalculate p
	p = p + (X1 + X2 + 3791) / 16;

	// Store result
	psInst->pressure = p;
}

void ConfigureI2C(bool fastMode){

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

