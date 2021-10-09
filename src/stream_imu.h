/*
   Streaming support for IMUs

   Copyright (C) 2021 Simon D. Levy

   MIT License

 */

#pragma once

#ifdef NONEXTERN
#define EXTERN
#else
#define EXTERN extern
#endif

EXTERN bool stream_imuGotGyrometer;
EXTERN bool stream_imuGotQuaternion;
EXTERN float stream_imuGyrometerX;
EXTERN float stream_imuGyrometerY;
EXTERN float stream_imuGyrometerZ;
EXTERN float stream_imuQuaternionW;
EXTERN float stream_imuQuaternionX;
EXTERN float stream_imuQuaternionY;
EXTERN float stream_imuQuaternionZ;

void stream_startImu(void);
void stream_updateImu(void);
