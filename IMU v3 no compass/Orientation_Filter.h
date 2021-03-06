/*
 * Orientation_Filter.h
 *
 *  Created on: Jan 8, 2017
 *      Author: Matthew
 */

#ifndef ORIENTATION_FILTER_H_
#define ORIENTATION_FILTER_H_

//*********************************************************************
//
// Definition of empirically found constants and other useful constants
//
//*********************************************************************
#define GYRO_OFFSET_X -0.003
#define GYRO_OFFSET_Y -0.0008
#define GYRO_OFFSET_Z .0006

//*****************************************************************************
//
// Median Filter window size, in samples
//
//*****************************************************************************
#define WINDOW_LENGTH 15

//*****************************************************************************
//
// LPF Constant - between 0 and 1
//
//*****************************************************************************
#define LPF_BETA 0.025

//*********************************************************************
//
// Definition of variables
//
//*********************************************************************
typedef struct {
	float qCurrent[4];
	float qGyro[4];
	float qAccel[4];
	float window[3][WINDOW_LENGTH];
	float accelFiltered[3];
	float LPaccelRaw[3];
	float yaw;
	float roll;
	float pitch;
	float Delta_T;
} OFilter;


//*********************************************************************
//
// Function Prototypes
//
//*********************************************************************
// Startup function called in main() that starts the Orientation filter
//	- sets time period between measurements, dt
//	- obtain starting orientation from mag and accel sensors
extern void Orientation_Filter_Init(OFilter *Filter, float dt);
// pass gyro, accel, and mag readings to get new orientation update
extern void Orientation_Filter_Update(OFilter *Filter, float gyro[3], float accel[3], uint8_t noGyro);
extern void AccelQ(OFilter *Filter, float accel[3]);
extern void getQuaternion(OFilter *Filter, float quat[4]);
extern void getEuler(OFilter *Filter, float euler[3]);
extern void SLERP(OFilter *Filter, float qOut[4], float qIn1[4], float qIn2[4], float gain);
extern void medianFilter(OFilter *Filter, float accelFiltered[3], float accelRaw[3], float window[3][WINDOW_LENGTH]);


#endif /* ORIENTATION_FILTER_H_ */
