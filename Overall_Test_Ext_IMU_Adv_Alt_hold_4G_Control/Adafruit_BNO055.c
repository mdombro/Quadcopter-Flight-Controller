/*
 * Adafruit_BNO055.c
 *
 *  Created on: Mar 25, 2017
 *      Author: Matthew
 */

#include <stdbool.h>
#include <stdint.h>
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "sensorlib/i2cm_drv.h"
#include "driverlib/pin_map.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
#include "driverlib/fpu.h"
#include "Adafruit_BNO055.h"

#define NULL ((char *)0)

//*****************************************************************************
//
//   PROTO-TYPES
//
//*****************************************************************************
void delay(uint32_t ms);
static void BNO055_callback(void *a, uint8_t b);
void I2CWait(void);

//*****************************************************************************
//
// Global instance structure for the I2C master driver.
//
//*****************************************************************************
tI2CMInstance g_sI2CInst;

tI2CMWrite8 sWriteState;

volatile uint8_t CHIP_I2C_STATUS = 0;

void Adafruit_BNO055_init(Adafruit_BNO055 *BNO, int32_t sensorID, uint8_t address) {
	BNO->_sensorID = sensorID;
	BNO->_address = address;

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

	IntMasterEnable();

	//
	// Initialize I2C3 peripheral.
	//
	I2CMInit(&g_sI2CInst, I2C3_BASE, INT_I2C3, 0xff, 0xff, SysCtlClockGet());
}

bool begin(Adafruit_BNO055 *BNO, adafruit_bno055_opmode_t mode) {
	// I2C init commands

	/* Make sure we have the right device */
	  uint8_t id = read8(BNO055_CHIP_ID_ADDR);
	  if(id != BNO055_ID)
	  {
	    delay(1000); // hold on for boot
	    id = read8(BNO055_CHIP_ID_ADDR);
	    if(id != BNO055_ID) {
	      return false;  // still not? ok bail
	    }
	  }

	  /* Switch to config mode (just in case since this is the default) */
	  setMode(BNO, OPERATION_MODE_CONFIG);
	  delay(10);
	  /* Reset */
	  write8(BNO055_SYS_TRIGGER_ADDR, 0x20);
	  I2CWait();
	  while (read8(BNO055_CHIP_ID_ADDR) != BNO055_ID)
	  {
	    delay(10);
	  }
	  delay(50);

	  /* Set to normal power mode */
	  write8(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
	  I2CWait();
	  delay(10);

	  write8(BNO055_PAGE_ID_ADDR, 0);
	  I2CWait();
	  /* Set the output units */

	  uint8_t unitsel = (0 << 7) | // Orientation = Android
	                    (0 << 4) | // Temperature = Celsius
	                    (0 << 2) | // Euler = Degrees
	                    (0 << 1) | // Gyro = Rads
	                    (0 << 0);  // Accelerometer = m/s^2
	  write8(BNO055_UNIT_SEL_ADDR, unitsel);
	  I2CWait();

	  /* Configure axis mapping (see section 3.4) */
	  /*
	  write8(BNO055_AXIS_MAP_CONFIG_ADDR, REMAP_CONFIG_P2); // P0-P7, Default is P1
	  delay(10);
	  write8(BNO055_AXIS_MAP_SIGN_ADDR, REMAP_SIGN_P2); // P0-P7, Default is P1
	  delay(10);
	  */

	  //write8(BNO055_SYS_TRIGGER_ADDR, 0x0);
	  //I2CWait();
	  delay(100);
	  /* Set the requested operating mode (see section 3.3) */
	  setMode(BNO, mode);
	  delay(20);

	  return true;
}

/**************************************************************************/
/*!
    @brief  Puts the chip in the specified operating mode
*/
/**************************************************************************/
void setMode(Adafruit_BNO055 *BNO, adafruit_bno055_opmode_t mode)
{
	BNO->_mode = mode;
	delay(100);
	write8(BNO055_OPR_MODE_ADDR, mode);
	I2CWait();
}


/**************************************************************************/
/*!
    @brief  Use the external 32.768KHz crystal
*/
/**************************************************************************/
void setExtCrystalUse(Adafruit_BNO055 *BNO, bool usextal)
{
  adafruit_bno055_opmode_t modeback = BNO->_mode;

  /* Switch to config mode (just in case since this is the default) */
  setMode(BNO, OPERATION_MODE_CONFIG);
  delay(25);
  write8(BNO055_PAGE_ID_ADDR, 0);
  I2CWait();
  if (usextal) {
    write8(BNO055_SYS_TRIGGER_ADDR, 0x80);
    I2CWait();
  } else {
    write8(BNO055_SYS_TRIGGER_ADDR, 0x00);
    I2CWait();
  }
  delay(10);
  /* Set the requested operating mode (see section 3.3) */
  setMode(BNO, modeback);
  delay(50);
}

/**************************************************************************/
/*!
    @brief  Gets the latest system status info
*/
/**************************************************************************/
void getSystemStatus(Adafruit_BNO055 *BNO, uint8_t *system_status, uint8_t *self_test_result, uint8_t *system_error)
{
  write8(BNO055_PAGE_ID_ADDR, 0);
  I2CWait();

  /* System Status (see section 4.3.58)
     ---------------------------------
     0 = Idle
     1 = System Error
     2 = Initializing Peripherals
     3 = System Iniitalization
     4 = Executing Self-Test
     5 = Sensor fusio algorithm running
     6 = System running without fusion algorithms */

  if (system_status != 0)
    *system_status    = read8(BNO055_SYS_STAT_ADDR);

  /* Self Test Results (see section )
     --------------------------------
     1 = test passed, 0 = test failed

     Bit 0 = Accelerometer self test
     Bit 1 = Magnetometer self test
     Bit 2 = Gyroscope self test
     Bit 3 = MCU self test

     0x0F = all good! */

  if (self_test_result != 0)
    *self_test_result = read8(BNO055_SELFTEST_RESULT_ADDR);

  /* System Error (see section 4.3.59)
     ---------------------------------
     0 = No error
     1 = Peripheral initialization error
     2 = System initialization error
     3 = Self test result failed
     4 = Register map value out of range
     5 = Register map address out of range
     6 = Register map write error
     7 = BNO low power mode not available for selected operat ion mode
     8 = Accelerometer power mode not available
     9 = Fusion algorithm configuration error
     A = Sensor configuration error */

  if (system_error != 0)
    *system_error     = read8(BNO055_SYS_ERR_ADDR);

  //delay(200);
}

void getEulers(Adafruit_BNO055 *BNO, float *xyz) {
	uint8_t buffer[6];
	uint8_t i;
	for (i = 0; i < 6; i++) {
		buffer[i] = 0;
	}
	int16_t x, y ,z;
	x = 0;
	y = 0;
	z = 0;

	readLen(BNO055_EULER_H_LSB_ADDR, buffer, 6);
	I2CWait();


	x = ((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8);
	y = ((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8);
	z = ((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8);

	xyz[0] = ((float)x)/16.0;
	xyz[1] = ((float)y)/16.0;
	xyz[2] = ((float)z)/16.0;
}

void getGyros(Adafruit_BNO055 *BNO, float *xyz) {
	uint8_t buffer[6];
	uint8_t i;
	for (i = 0; i < 6; i++) {
		buffer[i] = 0;
	}
	int16_t x, y ,z;
	x = 0;
	y = 0;
	z = 0;

	readLen(BNO055_GYRO_DATA_X_LSB_ADDR, buffer, 6);
	I2CWait();

	x = ((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8);
	y = ((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8);
	z = ((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8);

	xyz[0] = ((float)x)/16.0;
	xyz[1] = ((float)y)/16.0;
	xyz[2] = ((float)z)/16.0;

}

/**************************************************************************/
/*!
@brief  Writes to the sensor's offset registers from an offset struct
*/
/**************************************************************************/
void setSensorOffsets(Adafruit_BNO055 *BNO)
{
    adafruit_bno055_opmode_t lastMode = BNO->_mode;
    setMode(BNO, OPERATION_MODE_CONFIG);
    delay(25);

    write8(ACCEL_OFFSET_X_LSB_ADDR, (accel_x) & 0x0FF);
    I2CWait();
    write8(ACCEL_OFFSET_X_MSB_ADDR, (accel_x >> 8) & 0x0FF);
    I2CWait();
    write8(ACCEL_OFFSET_Y_LSB_ADDR, (accel_y) & 0x0FF);
    I2CWait();
    write8(ACCEL_OFFSET_Y_MSB_ADDR, (accel_y >> 8) & 0x0FF);
    I2CWait();
    write8(ACCEL_OFFSET_Z_LSB_ADDR, (accel_z) & 0x0FF);
    I2CWait();
    write8(ACCEL_OFFSET_Z_MSB_ADDR, (accel_z >> 8) & 0x0FF);
    I2CWait();

    write8(GYRO_OFFSET_X_LSB_ADDR, (gyro_x) & 0x0FF);
    I2CWait();
    write8(GYRO_OFFSET_X_MSB_ADDR, (gyro_x >> 8) & 0x0FF);
    I2CWait();
    write8(GYRO_OFFSET_Y_LSB_ADDR, (gyro_y) & 0x0FF);
    I2CWait();
    write8(GYRO_OFFSET_Y_MSB_ADDR, (gyro_y >> 8) & 0x0FF);
    I2CWait();
    write8(GYRO_OFFSET_Z_LSB_ADDR, (gyro_z) & 0x0FF);
    I2CWait();
    write8(GYRO_OFFSET_Z_MSB_ADDR, (gyro_z >> 8) & 0x0FF);
    I2CWait();

    write8(MAG_OFFSET_X_LSB_ADDR, (mag_x) & 0x0FF);
    I2CWait();
    write8(MAG_OFFSET_X_MSB_ADDR, (mag_x >> 8) & 0x0FF);
    I2CWait();
    write8(MAG_OFFSET_Y_LSB_ADDR, (mag_y) & 0x0FF);
    I2CWait();
    write8(MAG_OFFSET_Y_MSB_ADDR, (mag_y >> 8) & 0x0FF);
    I2CWait();
    write8(MAG_OFFSET_Z_LSB_ADDR, (mag_z) & 0x0FF);
    I2CWait();
    write8(MAG_OFFSET_Z_MSB_ADDR, (mag_z >> 8) & 0x0FF);
    I2CWait();

    write8(ACCEL_RADIUS_LSB_ADDR, (accel_r) & 0x0FF);
    I2CWait();
    write8(ACCEL_RADIUS_MSB_ADDR, (accel_r >> 8) & 0x0FF);
    I2CWait();

    write8(MAG_RADIUS_LSB_ADDR, (mag_r) & 0x0FF);
    I2CWait();
    write8(MAG_RADIUS_MSB_ADDR, (mag_r >> 8) & 0x0FF);
    I2CWait();

    setMode(BNO, lastMode);
}

/**************************************************************************/
/*!
    @brief  Gets current calibration state.  Each value should be a uint8_t
            pointer and it will be set to 0 if not calibrated and 3 if
            fully calibrated.
*/
/**************************************************************************/
void getCalibration(uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag) {
  uint8_t calData = read8(BNO055_CALIB_STAT_ADDR);
  if (sys != NULL) {
    *sys = (calData >> 6) & 0x03;
  }
  if (gyro != NULL) {
    *gyro = (calData >> 4) & 0x03;
  }
  if (accel != NULL) {
    *accel = (calData >> 2) & 0x03;
  }
  if (mag != NULL) {
    *mag = calData & 0x03;
  }
}

/**************************************************************************/
/*!
    @brief  Gets the temperature in degrees celsius
*/
/**************************************************************************/
int8_t getTemp(void)
{
  int8_t temp = (int8_t)(read8(BNO055_TEMP_ADDR));
  return temp;
}

/**************************************************************************/
/*!
    @brief  Writes an 8 bit value over I2C
*/
/**************************************************************************/
bool write8(adafruit_bno055_reg_t reg, uint8_t value)
{
	I2CMWrite8(&sWriteState, &g_sI2CInst, BNO055_ADDRESS_A, reg, &value, 1, BNO055_callback, 0);
	return true;
}

uint8_t read8(adafruit_bno055_reg_t reg )
{
  uint8_t value = 0x0;
  uint8_t _reg = reg;

  uint8_t v = I2CMRead(&g_sI2CInst, BNO055_ADDRESS_A, &_reg, 1, &value, 1, BNO055_callback, 0);
  I2CWait();
  return value;
}

bool readLen(adafruit_bno055_reg_t reg, uint8_t * buffer, uint16_t len)
{
	uint8_t _reg = reg;
	I2CMRead(&g_sI2CInst, BNO055_ADDRESS_A, &_reg, 1, buffer, len, BNO055_callback, 0);
	return true;
}

void delay(uint32_t ms) {
	SysCtlDelay(ms*40000);
}

void
I2CIntHandler(void)
{
    //
    // Pass through to the I2CM interrupt handler provided by sensor library.
    // This is required to be at application level so that I2CMIntHandler can
    // receive the instance structure pointer as an argument.
    //
    I2CMIntHandler(&g_sI2CInst);
}

static void BNO055_callback(void *a, uint8_t b) {
	CHIP_I2C_STATUS = 1;
}

void
I2CWait(void) {
	while(CHIP_I2C_STATUS == 0) {
	}
	CHIP_I2C_STATUS = 0;
}

