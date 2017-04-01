/*
 * PulsedLight_driver.c
 *
 *  Created on: Mar 31, 2017
 *      Author: SeniorDesign
 */

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "inc/tm4c123gh6pm.h"
#include "PulsedLight_driver.h"

void PulsedLightInit() {
	InitI2C0();
	I2CSend(PULSEDLIGHT_I2C_WRITE_ADDRESS,0x00,0x04);
	I2CSend(PULSEDLIGHT_I2C_WRITE_ADDRESS,PULSEDLIGHT_OFFSET_REG,0b11111100);
}

int16_t ReadDistance()
{
	// Integer to store data
	uint16_t Dist = 0;

	// Prepare Lidar for reading
	I2CSend(PULSEDLIGHT_I2C_WRITE_ADDRESS,0x00,0x04);

	//wait for 20 ms
	//SysCtlDelay(SysCtlClockGet() / (160));

	// Read Data
	Dist =  I2CReceive(PULSEDLIGHT_I2C_READ_ADDRESS,PULSEDLIGHT_I2C_WRITE_ADDRESS,PULSEDLIGHT_DISTANCE_REG);

    return Dist;
}

uint16_t I2CReceive(uint16_t ReadAddr,uint16_t WriteAddr, uint8_t reg){
	// Integer to store data.
	uint16_t Data = 0;

	//Initialize the i2c for writnign to slave device
	I2CMasterSlaveAddrSet(I2C0_BASE, WriteAddr >> 1, false);

	//Register to be read
	I2CMasterDataPut(I2C0_BASE, reg);

	//Sends cotroll - and register adress -byte to slave device
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);

	//Wait until slave device finished it's transaction.
	while(I2CMasterBusy(I2C0_BASE));

	//Initialize the i2c to read from slave device
	I2CMasterSlaveAddrSet(I2C0_BASE, ReadAddr>>1, true);

	//Read first Byte
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);

	//Wait until slave device finished it's transaction
	while(I2CMasterBusy(I2C0_BASE));

	// Read 8 MSB
	Data = I2CMasterDataGet(I2C0_BASE);

	// Shift Data
	Data = Data << 8;

	// Read last 8 Byte
	 I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);

	//Wait until slave device finished it's transaction
	 while(I2CMasterBusy(I2C0_BASE));

	// Shift in LSB
	 Data |= I2CMasterDataGet(I2C0_BASE);

	// If error occur store messg
	char test = I2CMasterErr(I2C0_BASE);

	// Return Data
	return Data;
}
void InitI2C0(void){

	//Enable i2c module 1
	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

	//Reset the i2c module 1
	SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);

	//Enable the GPIO periph that contains I2C0
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

	//Configure the pin muxing for i2c1, functions on port A6,A7.
	GPIOPinConfigure(GPIO_PB2_I2C0SCL);
	GPIOPinConfigure(GPIO_PB3_I2C0SDA);

	//Select i2c funciton on pins, SCL & SDA.
	GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
	GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

	//Initialize the i2c master block, I2C module data 1 pin PA(7).
	I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);

	// Enable internal pull-up resistor
	GPIO_PORTB_PUR_R = GPIO_PIN_7;

	//Clear i2c FIFOs
	HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000;

	// Enable line
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_2);
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 4);
}


uint16_t I2CSend(uint16_t WriteAddr, uint8_t reg, uint8_t value){
	//Writing register adress to slave device
	I2CMasterSlaveAddrSet(I2C0_BASE, WriteAddr >> 1, false);

	//Register to be read
	I2CMasterDataPut(I2C0_BASE, reg);

	//Sends cotroll - and register adress -byte to slave device
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);

	//Wait until slave device finished it's transaction
	while(I2CMasterBusy(I2C0_BASE));

	//Register adress to slave device
	I2CMasterSlaveAddrSet(I2C0_BASE, WriteAddr >> 1, false);

	// Value to send.
	I2CMasterDataPut(I2C0_BASE, value);

	//Sends cotroll - and register adress -byte to slave device
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);

	// //Wait until slave device finished it's transaction
	while(I2CMasterBusy(I2C0_BASE));

	// If error occur return messg.
	return I2CMasterErr(I2C0_BASE);
}
