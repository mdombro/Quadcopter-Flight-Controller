/*
 * Orientation_Filter.c
 *
 *  Created on: Jan 8, 2017
 *      Author: Matthew
 */

#include <math.h>
#include <stdint.h>
#include "sensorlib/quaternion.h"
#include "sensorlib/vector.h"
#include "Orientation_Filter.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288419716939937510582
#endif

//*****************************************************************************
//
// SLERP Gain for averaging accel/mag quaternion
//
//*****************************************************************************
#define GAIN_SLERP_AVERAGING 0.50

//*****************************************************************************
//
// SLERP Gain for overall filter
//
//*****************************************************************************
#define GAIN_SLERP 0.98


// Set Delta_T
// Read Accel, Mag and produce the initial orientation estimate
void Orientation_Filter_Init(OFilter *Filter, float dt) {
	// set
	float magW[3];
	Filter->Delta_T = dt;
	Filter->qCurrent[0] = 1;
	Filter->qCurrent[1] = 0;
	Filter->qCurrent[2] = 0;
	Filter->qCurrent[3] = 0;
	Filter->accelCorrection[0] = 0;
	Filter->accelCorrection[1] = 0;
	Filter->accelCorrection[2] = 0;
//	CorrectMag(mag);
//	Normalize(mag);
//	RotateBtoW(Filter, magW, mag);
//	Normalize(accel);
//	AccelMagQ(Filter, accel, magW);  // initiatlize estimation with accel and mag
//	Filter->qCurrent[0] = Filter->qAccelMag[0];
//	Filter->qCurrent[1] = Filter->qAccelMag[1];
//	Filter->qCurrent[2] = Filter->qAccelMag[2];
//	Filter->qCurrent[3] = Filter->qAccelMag[3];

}

void Orientation_Filter_Update(OFilter *Filter, float gyro[3], float accel[3], float mag[3], uint8_t noGyro) {
	float magW[3];
	float td2 = Filter->Delta_T*0.5;
	float qTemp[4];
	float grav[3] = {0, 0, -1.0};

	Normalize(accel);
	InverseRotate(accel, Filter->qCurrent);
	Filter->rotatedaccel[0] = accel[0];
	Filter->rotatedaccel[1] = accel[1];
	Filter->rotatedaccel[2] = accel[2];
	// Low pass filter
	VectorCrossProduct(Filter->accelCorrection, accel, grav);
	Filter->accelCorrection[0] *= 5.0f;
	Filter->accelCorrection[1] *= 5.0f;
	Filter->accelCorrection[2] *= 5.0f;
	Rotate(Filter->accelCorrection, Filter->qCurrent);
	Filter->accelCorrection[2] = 0;

	gyro[0] -= GYRO_OFFSET_X - Filter->accelCorrection[0];
	gyro[1] -= GYRO_OFFSET_Y - Filter->accelCorrection[1];
	gyro[2] -= GYRO_OFFSET_Z - Filter->accelCorrection[2];
	Filter->qGyro[1] = gyro[0]*td2;
	Filter->qGyro[2] = gyro[1]*td2;
	Filter->qGyro[3] = gyro[2]*td2;
	Filter->qGyro[0] = 1.0-0.5*(powf(Filter->qGyro[1],2)+powf(Filter->qGyro[2],2)+powf(Filter->qGyro[3],2));
	HamiltonProduct(qTemp, Filter->qCurrent, Filter->qGyro);
	Filter->qCurrent[0] = qTemp[0];
	Filter->qCurrent[1] = qTemp[1];
	Filter->qCurrent[2] = qTemp[2];
	Filter->qCurrent[3] = qTemp[3];
}

// Compute orientation quaternion based off accel and mag readings
// Reference Keeping a Good Attitude: A Quaternion-Based Orientation Filter for IMUs and MARGs
// Page 19311
void AccelMagQ(OFilter *Filter, float accel[3], float mag[3]) {

}

void CorrectMag(float mag[3]) {
	// Correct raw read mag values using values found empirically
	mag[0] -= 0.00000030;
	mag[1] -= 0.00000435;
	mag[2] -= 0.00000765;
	mag[0] *= 0.9895;
	mag[1] *= 0.9926;
	mag[2] *= 1.0185;
}

void Normalize(float vec[3]) {
	float norm = sqrt(powf(vec[0],2) + powf(vec[1],2) + powf(vec[2],2));
	vec[0] /= norm;
	vec[1] /= norm;
	vec[2] /= norm;
}

void Normalize4(float vec[4]) {
	float norm = sqrt(powf(vec[0],2) + powf(vec[1],2) + powf(vec[2],2) + powf(vec[3],2));
	vec[0] /= norm;
	vec[1] /= norm;
	vec[2] /= norm;
	vec[3] /= norm;
}

void RotateBtoW(OFilter *Filter, float vecW[3], float vecB[3]) {
	float qCurrentP[4] = {Filter->qCurrent[0], -1.0*Filter->qCurrent[1], -1.0*Filter->qCurrent[2], -1.0*Filter->qCurrent[3]};
	float vec4[4] = {0, vecB[0], vecB[1], vecB[2]};
	float qT[4];
	HamiltonProduct(qT, qCurrentP, vec4);
	HamiltonProduct(vec4, qT, Filter->qCurrent);
	vecW[0] = vec4[1];
	vecW[1] = vec4[2];
	vecW[2] = vec4[3];
	Filter->rMag[0] = vec4[1];
	Filter->rMag[1] = vec4[2];
	Filter->rMag[2] = vec4[3];
}

void Rotate(float vec[3], float quat[4]) {
	float quatP[4] = {quat[0], -1.0*quat[1], -1.0*quat[2], -1.0*quat[3]};
	float vec4[4] = {0, vec[0], vec[1], vec[2]};
	float qT[4];
	HamiltonProduct(qT, quat, vec4);
	HamiltonProduct(vec4, qT, quatP);
	vec[0] = vec4[1];
	vec[1] = vec4[2];
	vec[2] = vec4[3];
}

void InverseRotate(float vec[3], float quat[4]) {
	float quatP[4] = {quat[0], -1.0*quat[1], -1.0*quat[2], -1.0*quat[3]};
	float vec4[4] = {0, vec[0], vec[1], vec[2]};
	float qT[4];
	HamiltonProduct(qT, quatP, vec4);
	HamiltonProduct(vec4, qT, quat);
	vec[0] = vec4[1];
	vec[1] = vec4[2];
	vec[2] = vec4[3];
}



float Dot4(float In1[4], float In2[4]) {
	float dot = 0;
	uint8_t i;
	for (i = 0; i < 4; i++) {
		dot += In1[i]*In2[i];
	}
	return dot;
}

void DeInclinate(OFilter *Filter, float mag[3]) {
	float qCurrentP[4] = {Filter->qCurrent[0], -1.0*Filter->qCurrent[1], -1.0*Filter->qCurrent[2], -1.0*Filter->qCurrent[3]};
	float mag4[4] = {0, mag[0], mag[1], mag[2]};
	float qT[4];
	HamiltonProduct(qT, qCurrentP, mag4);
	HamiltonProduct(mag4, qT, Filter->qCurrent);
	mag4[3] = 0;
	//HamiltonProduct(qT, qCurrentP, mag4);
	//HamiltonProduct(mag4, qT, Filter->qCurrent);
	mag[0] = mag4[1];
	mag[1] = mag4[2];
	mag[2] = mag4[3];
	Filter->rMag[0] = mag4[1];
	Filter->rMag[1] = mag4[2];
	Filter->rMag[2] = mag4[3];
}

void getEuler(OFilter *Filter, float euler[3]) {
	euler[0] = atan2(2.0f*(Filter->qCurrent[0]*Filter->qCurrent[1]+Filter->qCurrent[2]*Filter->qCurrent[3]), 1.0f-2.0f*(Filter->qCurrent[1]*Filter->qCurrent[1]+Filter->qCurrent[2]*Filter->qCurrent[2]));
	float asin_arg = 2.0f*(Filter->qCurrent[0]*Filter->qCurrent[2]-Filter->qCurrent[3]*Filter->qCurrent[1]);
	asin_arg = asin_arg > 1.0f ? 1.0f : asin_arg;
	asin_arg = asin_arg < -1.0f ? -1.0f : asin_arg;
	euler[1] = asin(asin_arg);
	euler[2] = atan2(2.0f*(Filter->qCurrent[0]*Filter->qCurrent[3]+Filter->qCurrent[1]*Filter->qCurrent[2]), 1.0f-2.0f*(Filter->qCurrent[2]*Filter->qCurrent[2]+Filter->qCurrent[3]*Filter->qCurrent[3]));
	euler[0] *= 180.0/M_PI;
	euler[1] *= 180.0/M_PI;
	euler[2] *= 180.0/M_PI;
}

void getQuaternion(OFilter *Filter, float quat[4]) {
	quat[0] = Filter->qCurrent[0];
	quat[1] = Filter->qCurrent[1];
	quat[2] = Filter->qCurrent[2];
	quat[3] = Filter->qCurrent[3];
}

void HamiltonProduct(float out[4], float in1[4], float in2[4]) {
	out[0] = in1[0]*in2[0] - in1[1]*in2[1] - in1[2]*in2[2] - in1[3]*in2[3];
	out[1] = in1[0]*in2[1] + in1[1]*in2[0] + in1[2]*in2[3] - in1[3]*in2[2];
	out[2] = in1[0]*in2[2] - in1[1]*in2[3] + in1[2]*in2[0] + in1[3]*in2[1];
	out[3] = in1[0]*in2[3] + in1[1]*in2[2] - in1[2]*in2[1] + in1[3]*in2[0];
}

