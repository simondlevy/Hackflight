/**
 * Copyright (C) 2011-2022 Bitcraze AB, 2024 Simon D. Levy
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>

typedef union {
    struct {
        int16_t x;
        int16_t y;
        int16_t z;
    };
    int16_t axis[3];
} Axis3i16;

typedef union {
    struct {
        float x;
        float y;
        float z;
    };
    float axis[3];
} Axis3f;

struct vec3_s {
    uint32_t timestamp; // Timestamp when the data was computed
    float x;
    float y;
    float z;
};

typedef struct vec3_s point_t;

typedef struct quaternion_s {
    union {
        struct {
            float q0;
            float q1;
            float q2;
            float q3;
        };
        struct {
            float x;
            float y;
            float z;
            float w;
        };
    };
} quaternion_t;

typedef struct {

    float thrust;  // positve upward
    float roll;    // positive roll right
    float pitch;   // positive nose down
    float yaw;     // positive nose right

} demands_t;

typedef void (*mixFun_t)(
        const demands_t & demands, float motorvals[], uint8_t & count);


typedef void (*openLoopFun_t)(
        demands_t & demands, uint32_t & timestamp, bool & inHoverMode);

// From Eqn. (11) in Bouabdallah,  Murrieri, Siegwart (2004). 
// We use ENU coordinates based on 
// https://www.bitcraze.io/documentation/system/platform/cf2-coordinate-system
// Position in meters, velocity in meters/second, angles in degrees,
// angular velocity in degrees/second.
typedef struct {

    float x;       // positive forward
    float dx;      // positive forward
    float y;       // positive leftward
    float dy;      // positive leftward
    float z;       // positive upward
    float dz;      // positive upward
    float phi;     // positive roll right
    float dphi;    // positive roll right
    float theta;   // positive nose up
    float dtheta;  // positive nose up (opposite of gyro Y)
    float psi;     // positive nose left
    float dpsi;    // positive nose left

} vehicleState_t;

/* Data structure used by the stabilizer subsystem.
 * All have a timestamp to be set when the data is calculated.
 */

/* vector */
#define vec3d_size 3
typedef float vec3d[vec3d_size];
typedef float mat3d[vec3d_size][vec3d_size];

typedef struct vec3_s vector_t;

typedef enum {
    MeasurementSourceLocationService  = 0,
} measurementSource_t;

typedef struct tdoaMeasurement_s {
    union {
        point_t anchorPositions[2];
        struct {
            point_t anchorPositionA;
            point_t anchorPositionB;
        };
    };
    union {
        uint8_t anchorIds[2];
        struct {
            uint8_t anchorIdA;
            uint8_t anchorIdB;
        };
    };

    float distanceDiff;
    float stdDev;
} tdoaMeasurement_t;

typedef struct baro_s {
    float pressure;           // mbar
    float temperature;        // degree Celcius
    float asl;                // m (ASL = altitude above sea level)
} baro_t;

typedef struct positionMeasurement_s {
    union {
        struct {
            float x;
            float y;
            float z;
        };
        float pos[3];
    };
    float stdDev;
    measurementSource_t source;
} positionMeasurement_t;

typedef struct poseMeasurement_s {
    union {
        struct {
            float x;
            float y;
            float z;
        };
        float pos[3];
    };
    quaternion_t quat;
    float stdDevPos;
    float stdDevQuat;
} poseMeasurement_t;

typedef struct distanceMeasurement_s {
  union {
    struct {
      float x;
      float y;
      float z;
    };
    float pos[3];
  };
  uint8_t anchorId;
  float distance;
  float stdDev;
} distanceMeasurement_t;

typedef struct zDistance_s {
  uint32_t timestamp;
  float distance;           // m
} zDistance_t;

typedef struct sensorData_s {
  Axis3f acc;               // Gs
  Axis3f gyro;              // deg/s
  Axis3f mag;               // gauss
  baro_t baro;
  uint64_t interruptTimestamp;
} sensorData_t;

/** Estimate of position */
typedef struct estimate_s {
  uint32_t timestamp; // Timestamp when the data was computed

  point_t position;
} estimate_t;

/** Setpoint for althold */
typedef struct setpointZ_s {
  float z;
  bool isUpdate; // True = small update of setpoint, false = completely new
} setpointZ_t;

/** Flow measurement**/
typedef struct flowMeasurement_s {
  uint32_t timestamp;
  union {
    struct {
      float dpixelx;  // Accumulated pixel count x
      float dpixely;  // Accumulated pixel count y
    };
    float dpixel[2];  // Accumulated pixel count
  };
  float stdDevX;      // Measurement standard deviation
  float stdDevY;      // Measurement standard deviation
  float dt;           // Time during which pixels were accumulated
} flowMeasurement_t;


/** TOF measurement**/
typedef struct tofMeasurement_s {
  uint32_t timestamp;
  float distance;
  float stdDev;
} tofMeasurement_t;

/** Absolute height measurement */
typedef struct heightMeasurement_s {
  uint32_t timestamp;
  float height;
  float stdDev;
} heightMeasurement_t;

/** Yaw error measurement */
typedef struct {
  uint32_t timestamp;
  float yawError;
  float stdDev;
} yawErrorMeasurement_t;

/** Sweep angle measurement */
typedef struct {
  uint32_t timestamp;
  const vec3d* sensorPos;    // Sensor position in the CF reference frame
  const vec3d* rotorPos;     // Pos of rotor origin in global reference frame
  const mat3d* rotorRot;     // Rotor rotation matrix
  const mat3d* rotorRotInv;  // Inverted rotor rotation matrix
  uint8_t sensorId;
  uint8_t baseStationId;
  uint8_t sweepId;
  float t;                   // t is the tilt angle of the light plane on the rotor
  float measuredSweepAngle;
  float stdDev;
} sweepAngleMeasurement_t;

/** gyroscope measurement */
typedef struct
{
  Axis3f gyro; // deg/s, for legacy reasons
} gyroscopeMeasurement_t;

/** accelerometer measurement */
typedef struct
{
  Axis3f acc; // Gs, for legacy reasons
} accelerationMeasurement_t;

/** barometer measurement */
typedef struct
{
  baro_t baro; // for legacy reasons
} barometerMeasurement_t;

typedef struct {
  // position - mm
  int16_t x;
  int16_t y;
  int16_t z;
  // velocity - mm / sec
  int16_t vx;
  int16_t vy;
  int16_t vz;
  // acceleration - mm / sec^2
  int16_t ax;
  int16_t ay;
  int16_t az;
  // compressed quaternion, see quatcompress.h
  int32_t quat;
  // angular velocity - milliradians / sec
  int16_t rateRoll;
  int16_t ratePitch;
  int16_t rateYaw;
} stateCompressed_t;

typedef struct {
  // position - mm
  int16_t x;
  int16_t y;
  int16_t z;
  // velocity - mm / sec
  int16_t vx;
  int16_t vy;
  int16_t vz;
  // acceleration - mm / sec^2
  int16_t ax;
  int16_t ay;
  int16_t az;
} setpointCompressed_t;
