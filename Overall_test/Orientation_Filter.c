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
#define GAIN_SLERP 0.01

// Set Delta_T and quaternion estimation to null rotation
void Orientation_Filter_Init(OFilter *Filter, float dt) {
	Filter->Delta_T = dt;
	uint8_t i, j;
	for (i = 0; i < 3; i++) {
		for (j = 0; j < WINDOW_LENGTH; j++) {
			Filter->window[i][j] = 0;
		}
	}
	for (i = 0; i < 3; i++) {
		Filter->accelFiltered[i] = 0;
	}
	Filter->LPaccelRaw[0] = 0;
	Filter->LPaccelRaw[1] = 0;
	Filter->LPaccelRaw[2] = 0;
	Filter->qCurrent[0] = 1;
	Filter->qCurrent[1] = 0;
	Filter->qCurrent[2] = 0;
	Filter->qCurrent[3] = 0;
}

void Orientation_Filter_Update(OFilter *Filter, float gyro[3], float accel[3], uint8_t noGyro) {
	float td2 = Filter->Delta_T*0.5;
	float qTemp[4];
	medianFilter(Filter, Filter->accelFiltered, accel, Filter->window);
	if (!noGyro) {
		gyro[0] -= GYRO_OFFSET_X;
		gyro[1] -= GYRO_OFFSET_Y;
		gyro[2] -= GYRO_OFFSET_Z;
		Filter->qGyro[1] = gyro[0]*td2; //*6;
		Filter->qGyro[2] = gyro[1]*td2; //*10;
		Filter->qGyro[3] = gyro[2]*td2; //*8;
		Filter->qGyro[0] = 1.0-0.5*(powf(Filter->qGyro[1],2)+powf(Filter->qGyro[2],2)+powf(Filter->qGyro[3],2));
		HamiltonProduct(qTemp, Filter->qCurrent, Filter->qGyro);
		Filter->qCurrent[0] = qTemp[0];
		Filter->qCurrent[1] = qTemp[1];
		Filter->qCurrent[2] = qTemp[2];
		Filter->qCurrent[3] = qTemp[3];
		Normalize(Filter->accelFiltered);
		AccelQ(Filter, Filter->accelFiltered);
		SLERP(Filter, Filter->qCurrent, qTemp, Filter->qAccel, GAIN_SLERP);
	}
	else {
		Normalize(Filter->accelFiltered);
		AccelQ(Filter, Filter->accelFiltered);
		SLERP(Filter, qTemp, Filter->qCurrent, Filter->qAccel, GAIN_SLERP_AVERAGING);
		Filter->qCurrent[0] = qTemp[0];
		Filter->qCurrent[1] = qTemp[1];
		Filter->qCurrent[2] = qTemp[2];
		Filter->qCurrent[3] = qTemp[3];
	}
}

// Compute orientation quaternion based off accel and mag readings
// Reference Keeping a Good Attitude: A Quaternion-Based Orientation Filter for IMUs and MARGs
// Page 19311
void AccelQ(OFilter *Filter, float accel[3]) {
	float t1 = sqrt(2*(accel[2]+1));
	float t2 = sqrt(2*(1-accel[2]));
	if (accel[2] >= 0) {
		Filter->qAccel[0] = sqrt((accel[2]+1)/2);
		Filter->qAccel[1] = -accel[1]/t1;
		Filter->qAccel[2] = accel[0]/t1;
		Filter->qAccel[3] = Filter->qCurrent[3];
	}
//	float tmp[4];
//	HamiltonProduct(tmp, Filter->qAccel, Filter->qCurrent);
//	Filter->qAccel[0] = tmp[0];
//	Filter->qAccel[1] = tmp[1];
//	Filter->qAccel[2] = tmp[2];
//	Filter->qAccel[3] = tmp[3];
//	else {
//		Filter->qAccel[0] = -accel[1]/t2;
//		Filter->qAccel[1] = sqrt((1-accel[2])/2);
//		Filter->qAccel[2] = 0;
//		Filter->qAccel[3] = accel[0]/t2;
//	}
	//Filter->roll = 180*atan2(2*(Filter->qAccel[0]*Filter->qAccel[1] + Filter->qAccel[2]*Filter->qAccel[3]), 1.0f-2.0f*(powf(Filter->qAccel[1],2) + powf(Filter->qAccel[2],2)))/M_PI;
	//Filter->pitch = 180*asin(2.0f*(Filter->qAccel[0]*Filter->qAccel[2] - Filter->qAccel[3]*Filter->qAccel[1]))/M_PI;
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

float Dot4(float qIn1[4], float qIn2[4]) {
	return qIn1[0]*qIn2[0] + qIn1[1]*qIn2[1] + qIn1[2]*qIn2[2] + qIn1[3]*qIn2[3];
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

void medianFilter(OFilter *Filter, float accelFiltered[3], float accelRaw[3], float window[3][WINDOW_LENGTH]) {
	uint8_t i,j;
	// Light low passing to remove worst of spikes
	Filter->LPaccelRaw[0] = Filter->LPaccelRaw[0]-(LPF_BETA*(Filter->LPaccelRaw[0]-accelRaw[0]));
	Filter->LPaccelRaw[1] = Filter->LPaccelRaw[1]-(LPF_BETA*(Filter->LPaccelRaw[1]-accelRaw[1]));
	Filter->LPaccelRaw[2] = Filter->LPaccelRaw[2]-(LPF_BETA*(Filter->LPaccelRaw[2]-accelRaw[2]));
	for (i = 0; i < 3; i++) {
		for (j = 0; j < WINDOW_LENGTH-1; j++) {
			window[i][j] = window[i][j+1];
		}
	}
	window[0][WINDOW_LENGTH-1] = Filter->LPaccelRaw[0];
	window[1][WINDOW_LENGTH-1] = Filter->LPaccelRaw[1];
	window[2][WINDOW_LENGTH-1] = Filter->LPaccelRaw[2];
	float *windowP_x[WINDOW_LENGTH];
	for (i = 0; i < WINDOW_LENGTH; i++) {
		windowP_x[i] = window[0]+i;
	}
	float *windowP_y[WINDOW_LENGTH];
	for (i = 0; i < WINDOW_LENGTH; i++) {
		windowP_y[i] = window[1]+i;
	}
	float *windowP_z[WINDOW_LENGTH];
	for (i = 0; i < WINDOW_LENGTH; i++) {
		windowP_z[i] = window[2]+i;
	}
	sort(windowP_x, WINDOW_LENGTH);
	sort(windowP_y, WINDOW_LENGTH);
	sort(windowP_z, WINDOW_LENGTH);
	accelFiltered[0] = *(windowP_x[2]);
	accelFiltered[1] = *(windowP_y[2]);
	accelFiltered[2] = *(windowP_z[2]);
}

void sort(float *list[5], uint8_t size) {
	uint8_t i, j;
	for(i = 0; i < size; i++){
		for(j = i + 1; j < size; j++){
			if (*(list[j]) < *(list[i])) {
				void *temp = list[j];
				list[j] = list[i];
				list[i] = temp;
			}
		}
	}
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

