//*****************************************************************************
//
// quaternion.h - Prototypes for the quaternion functions.
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
// This is part of revision 2.1.3.156 of the Tiva Firmware Development Package.
//
//*****************************************************************************

#ifndef __SENSORLIB_QUATERNION_H__
#define __SENSORLIB_QUATERNION_H__

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// The index of the components in a quaternion vector.
//
//*****************************************************************************
#define Q_W                     0
#define Q_X                     1
#define Q_Y                     2
#define Q_Z                     3

//*****************************************************************************
//
// Prototypes.
//
//*****************************************************************************
extern void QuaternionFromEuler(float pfQOut[4], float fRollDeg,
                                float fPitchDeg, float fYawDeg);
extern float QuaternionMagnitude(float pfQIn[4]);
extern void QuaternionInverse(float pfQOut[4], float pfQIn[4]);
extern void QuaternionMult(float pfQOut[4], float pfQIn1[4], float pfQIn2[4]);
extern float QuaternionAngle(float pfQIn1[4], float pfQIn2[4]);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // __SENSORLIB_QUATERNION_H__
