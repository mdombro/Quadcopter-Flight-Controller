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
	if (!noGyro) {
		gyro[0] -= GYRO_OFFSET_X;
		gyro[1] -= GYRO_OFFSET_Y;
		gyro[2] -= GYRO_OFFSET_Z;
		Filter->qGyro[1] = gyro[0]*td2;
		Filter->qGyro[2] = gyro[1]*td2;
		Filter->qGyro[3] = gyro[2]*td2;
		Filter->qGyro[0] = 1.0-0.5*(powf(Filter->qGyro[1],2)+powf(Filter->qGyro[2],2)+powf(Filter->qGyro[3],2));
		HamiltonProduct(qTemp, Filter->qCurrent, Filter->qGyro);
//		Filter->qCurrent[0] = qTemp[0];
//		Filter->qCurrent[1] = qTemp[1];
//		Filter->qCurrent[2] = qTemp[2];
//		Filter->qCurrent[3] = qTemp[3];
		Filter->qCurrent[0] = Filter->qGyro[0];
		Filter->qCurrent[1] = Filter->qGyro[1];
		Filter->qCurrent[2] = Filter->qGyro[2];
		Filter->qCurrent[3] = Filter->qGyro[3];
		CorrectMag(mag);
		Normalize(mag);
		//RotateBtoW(Filter, magW, mag);
		Normalize(accel);
		AccelMagQ(Filter, accel, mag);
		//SLERP(Filter, Filter->qCurrent, qTemp, Filter->qAccelMag, 0.90);
	}
	else {
		CorrectMag(mag);
		Normalize(mag);
		//RotateBtoW(Filter, magW, mag);
		Normalize(accel);
		AccelMagQ(Filter, accel, mag);
		SLERP(Filter, qTemp, Filter->qCurrent, Filter->qAccelMag, 0.50);
		Filter->qCurrent[0] = qTemp[0];
		Filter->qCurrent[1] = qTemp[1];
		Filter->qCurrent[2] = qTemp[2];
		Filter->qCurrent[3] = qTemp[3];
	}
}

// Compute orientation quaternion based off accel and mag readings
// Reference Keeping a Good Attitude: A Quaternion-Based Orientation Filter for IMUs and MARGs
// Page 19311
void AccelMagQ(OFilter *Filter, float accel[3], float mag[3]) {
	float t1 = sqrt(2*(accel[2]+1));
	float t2 = sqrt(2*(1-accel[2]));
	if (accel[2] >= 0) {
		Filter->qAccel[0] = sqrt((accel[2]+1)/2);
		Filter->qAccel[1] = -accel[1]/t1;
		Filter->qAccel[2] = accel[0]/t1;
		Filter->qAccel[3] = 0;
	}
	else {
		Filter->qAccel[0] = -accel[1]/t2;
		Filter->qAccel[1] = sqrt((1-accel[2])/2);
		Filter->qAccel[2] = 0;
		Filter->qAccel[3] = accel[0]/t2;
	}
	Filter->roll = 180*atan2(2*(Filter->qAccel[0]*Filter->qAccel[1] + Filter->qAccel[2]*Filter->qAccel[3]), 1.0f-2.0f*(powf(Filter->qAccel[1],2) + powf(Filter->qAccel[2],2)))/M_PI;
	Filter->pitch = 180*asin(2.0f*(Filter->qAccel[0]*Filter->qAccel[2] - Filter->qAccel[3]*Filter->qAccel[1]))/M_PI;


	// Trying direct yaw compute approach
//	InverseRotate(mag, Filter->qAccel);
//	Normalize(mag);
//	Filter->rMag[0] = mag[0];
//	Filter->rMag[1] = mag[1];
//	Filter->rMag[2] = mag[2];
//	float yaw = atan2(mag[1], mag[0]);
//	float tt0 = cos(yaw*0.5f);
//	float tt1 = sin(yaw*0.5f);
//	Filter->qMag[0] = tt0;
//	Filter->qMag[1] = 0;
//	Filter->qMag[2] = 0;
//	Filter->qMag[3] = tt1;

	Filter->nonComMag[0] = mag[0];
	Filter->nonComMag[1] = mag[1];
	Filter->nonComMag[2] = mag[2];
	InverseRotate(mag, Filter->qAccel);
	Filter->ComMag[0] = mag[0];
	Filter->ComMag[1] = mag[1];
	Filter->ComMag[2] = mag[2];
	Filter->planarMag[0] = mag[0];
	Filter->planarMag[1] = mag[1];
	Filter->planarMag[2] = mag[2];
	Normalize(mag);
	Filter->rMag[0] = mag[0];
	Filter->rMag[1] = mag[1];
	Filter->rMag[2] = mag[2];
	float lambda = powf(mag[0],2)+powf(mag[1],2);
	float sqrt_lambda = sqrt(lambda);
	float sqrt_2_lambda = sqrt(2*lambda);
	float sqrt_2 = sqrt(2);
	float tm1 = sqrt(lambda+mag[0]*sqrt_lambda);
	float tm2 = sqrt(lambda-mag[0]*sqrt_lambda);
	if (mag[0] >= 0) {
		Filter->qMag[0] = tm1/sqrt_2_lambda;
		Filter->qMag[1] = 0;
		Filter->qMag[2] = 0;
		Filter->qMag[3] = mag[1]/(sqrt_2*tm1);
	}
	else {
		Filter->qMag[0] = mag[1]/(sqrt_2*tm2);
		Filter->qMag[1] = 0;
		Filter->qMag[2] = 0;
		Filter->qMag[3] = tm2/sqrt_2_lambda;
	}
	Normalize4(Filter->qMag);


	Filter->yaw = 180*atan2(2*(Filter->qMag[0]*Filter->qMag[3]), 1.0f - 2*(powf(Filter->qMag[3],2)))/M_PI;


//	Filter->qMag[0] = 1;
//	Filter->qMag[1] = 0;
//	Filter->qMag[2] = 0;
//	Filter->qMag[3] = 0;
	HamiltonProduct(Filter->qAccelMag, Filter->qMag, Filter->qAccel);

//	Filter->qAccelMag[0] = Filter->qAccel[0];
//	Filter->qAccelMag[1] = Filter->qAccel[1];
//	Filter->qAccelMag[2] = Filter->qAccel[2];
//	Filter->qAccelMag[3] = Filter->qAccel[3];
	Normalize4(Filter->qAccelMag);
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

void SLERP(OFilter *Filter, float qOut[4], float qIn1[4], float qIn2[4], float gain) {
	// SLERP algorithm reference: http://www.willperone.net/Code/quaternion.php
	float qT[4];
	float dot = Dot4(qIn1, qIn2);
	uint8_t i;

	// dot == cos(theta)
	// If dot < 0, q1 and q2 > 90 degrees apart
	// So invert one of the quaternions to reduce rotation size
	if (dot < 0) {
		dot = -dot;
		qT[0] = -qIn2[0];
		qT[1] = -qIn2[1];
		qT[2] = -qIn2[2];
		qT[3] = -qIn2[3];
	}
	else {
		qT[0] = qIn2[0];
		qT[1] = qIn2[1];
		qT[2] = qIn2[2];
		qT[3] = qIn2[3];
	}
	if (dot < 0.95f) {
		float angle = acos(dot);
		float t1 = sin(angle*(1.0-gain));
		float t2 = sin(angle*gain);
		float t3 = sin(angle);
		for (i = 0; i < 4; i++) {
			qOut[i] = (qIn1[i]*t1 + qT[i]*t2)/t3;
		}
	}
	else {
		float omg = (1-gain);
		for (i = 0; i < 4; i++) {
			qOut[i] = qIn1[i]*omg + qT[i]*gain;
		}
		Normalize(qOut);
	}
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

