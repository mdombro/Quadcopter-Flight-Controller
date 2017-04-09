/*
 * PulsedLight_driver.h
 *
 *  Created on: Mar 31, 2017
 *      Author: SeniorDesign
 */

#ifndef PULSEDLIGHT_DRIVER_H_
#define PULSEDLIGHT_DRIVER_H_

#define PULSEDLIGHT_I2C_WRITE_ADDRESS 0xc4
#define PULSEDLIGHT_I2C_READ_ADDRESS 0xc5
#define PULSEDLIGHT_DISTANCE_REG 0x8f
#define PULSEDLIGHT_OFFSET_REG 0x13

extern void PulsedLightInit();
extern int16_t ReadDistance();
extern uint16_t I2CReceive(uint16_t ReadAddr,uint16_t WriteAddr, uint8_t reg);
extern void InitI2C0(void);
extern uint16_t I2CSend(uint16_t WriteAddr, uint8_t reg, uint8_t value);


#endif /* PULSEDLIGHT_DRIVER_H_ */
