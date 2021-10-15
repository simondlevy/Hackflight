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
