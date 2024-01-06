/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2017 Bitcraze AB
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
 *
 *
 */
#include <stdint.h>
#include <string.h>

#include <num.hpp>
#include <physicalConstants.h>
#include <quatcompress.h>

#include <commander.hpp>

#include "crtp.h"
#include "crtp_commander.h"

/* The generic commander format contains a packet type and data that has to be
 * decoded into a setpoint_t structure. The aim is to make it future-proof
 * by easily allowing the addition of new packets for future use cases.
 *
 * The packet format is:
 * +------+==========================+
 * | TYPE |     DATA                 |
 * +------+==========================+
 *
 * The type is defined bellow together with a decoder function that should take
 * the data buffer in and fill up a setpoint_t structure.
 * The maximum data size is 29 bytes.
 */

/* To add a new packet:
 *   1 - Add a new type in the packetType_e enum.
 *   2 - Implement a decoder function with good documentation about the data
 *       structure and the intent of the packet.
 *   3 - Add the decoder function to the packetDecoders array.
 *   4 - Create a new params group for your handler if necessary
 *   5 - Pull-request your change :-)
 */

typedef void (*packetDecoder_t)(setpoint_t *setpoint, uint8_t type, const void *data, size_t datalen);

/* ---===== 1 - packetType_e enum =====--- */
enum packet_type {
  stopType          = 0,
  velocityWorldType = 1,
  zDistanceType     = 2,
  cppmEmuType       = 3,
  altHoldType       = 4,
  hoverType         = 5,
  fullStateType     = 6,
  positionType      = 7,
};

/* ---===== 2 - Decoding functions =====--- */
/* The setpoint structure is reinitialized to 0 before being passed to the
 * functions
 */

/* stopDecoder
 * Keeps setpoint to 0: stops the motors and fall
 */
static void stopDecoder(setpoint_t *setpoint, uint8_t type, const void *data, size_t datalen)
{
  return;
}

/* velocityDecoder
 * Set the Crazyflie velocity in the world coordinate system
 */
struct velocityPacket_s {
  float vx;        // m in the world frame of reference
  float vy;        // ...
  float vz;        // ...
  float yawrate;  // deg/s
} __attribute__((packed));

static void velocityDecoder(
        setpoint_t *setpoint, uint8_t type, const void *data, size_t datalen)
{
  const struct velocityPacket_s *values = (velocityPacket_s *)data;

  //ASSERT(datalen == sizeof(struct velocityPacket_s));

  setpoint->mode.x = modeVelocity;
  setpoint->mode.y = modeVelocity;
  setpoint->mode.z = modeVelocity;

  setpoint->velocity.x = values->vx;
  setpoint->velocity.y = values->vy;
  setpoint->velocity.z = values->vz;

  setpoint->mode.yaw = modeVelocity;

  setpoint->attitudeRate.yaw = values->yawrate;
}

/* zDistanceDecoder
 * Set the Crazyflie absolute height and roll/pitch angles
 */
struct zDistancePacket_s {
  float roll;            // deg
  float pitch;           // ...
  float yawrate;         // deg/s
  float zDistance;        // m in the world frame of reference
} __attribute__((packed));
static void zDistanceDecoder(setpoint_t *setpoint, uint8_t type, const void *data, size_t datalen)
{
  const struct zDistancePacket_s *values = (zDistancePacket_s *)data;


  //ASSERT(datalen == sizeof(struct zDistancePacket_s));

  setpoint->mode.z = modeAbs;

  setpoint->position.z = values->zDistance;

  setpoint->mode.yaw = modeVelocity;

  setpoint->attitudeRate.yaw = values->yawrate;


  setpoint->mode.roll = modeAbs;
  setpoint->mode.pitch = modeAbs;

  setpoint->attitude.roll = values->roll;
  setpoint->attitude.pitch = values->pitch;
}

static inline float getChannelUnitMultiplier(uint16_t channelValue, uint16_t channelMidpoint, uint16_t channelRange)
{
  // Compute a float from -1 to 1 based on the RC channel value, midpoint, and
  // total range magnitude
  return ((float)channelValue - (float)channelMidpoint) / (float)channelRange;
}

static void cppmEmuDecoder(
        setpoint_t *setpoint, uint8_t type, const void *data, size_t datalen)
{
}

/* altHoldDecoder
 * Set the Crazyflie vertical velocity and roll/pitch angle
 */
struct altHoldPacket_s {
  float roll;            // rad
  float pitch;           // ...
  float yawrate;         // deg/s
  float zVelocity;       // m/s in the world frame of reference
} __attribute__((packed));

static void altHoldDecoder(
        setpoint_t *setpoint, uint8_t type, const void *data, size_t datalen)
{
  const struct altHoldPacket_s *values = (altHoldPacket_s *)data;

  //ASSERT(datalen == sizeof(struct altHoldPacket_s));


  setpoint->mode.z = modeVelocity;

  setpoint->velocity.z = values->zVelocity;


  setpoint->mode.yaw = modeVelocity;

  setpoint->attitudeRate.yaw = values->yawrate;


  setpoint->mode.roll = modeAbs;
  setpoint->mode.pitch = modeAbs;

  setpoint->attitude.roll = values->roll;
  setpoint->attitude.pitch = values->pitch;
}

/* hoverDecoder
 * Set the Crazyflie absolute height and velocity in the body coordinate system
 */
struct hoverPacket_s {
  float vx;           // m/s in the body frame of reference
  float vy;           // ...
  float yawrate;      // deg/s
  float zDistance;    // m in the world frame of reference
} __attribute__((packed));
static void hoverDecoder(setpoint_t *setpoint, uint8_t type, const void *data, size_t datalen)
{
  const struct hoverPacket_s *values = (hoverPacket_s *)data;

  //ASSERT(datalen == sizeof(struct hoverPacket_s));

  setpoint->mode.z = modeAbs;
  setpoint->position.z = values->zDistance;


  setpoint->mode.yaw = modeVelocity;
  setpoint->attitudeRate.yaw = values->yawrate;


  setpoint->mode.x = modeVelocity;
  setpoint->mode.y = modeVelocity;
  setpoint->velocity.x = values->vx;
  setpoint->velocity.y = values->vy;

  setpoint->velocity_body = true;
}

struct fullStatePacket_s {
  int16_t x;         // position - mm
  int16_t y;
  int16_t z;
  int16_t vx;        // velocity - mm / sec
  int16_t vy;
  int16_t vz;
  int16_t ax;        // acceleration - mm / sec^2
  int16_t ay;
  int16_t az;
  int32_t quat;      // compressed quaternion, see quatcompress.h
  int16_t rateRoll;  // angular velocity - milliradians / sec
  int16_t ratePitch; //  (NOTE: limits to about 5 full circles per sec.
  int16_t rateYaw;   //   may not be enough for extremely aggressive flight.)
} __attribute__((packed));

static void fullStateDecoder(
        setpoint_t *setpoint, uint8_t type, const void *data, size_t datalen)
{
  const struct fullStatePacket_s *values = (fullStatePacket_s *)data;

  //ASSERT(datalen == sizeof(struct fullStatePacket_s));

  #define UNPACK(x) \
  setpoint->mode.x = modeAbs; \
  setpoint->position.x = values->x / 1000.0f; \
  setpoint->velocity.x = (values->v ## x) / 1000.0f; \
  setpoint->acceleration.x = (values->a ## x) / 1000.0f; \

  UNPACK(x)
  UNPACK(y)
  UNPACK(z)
  #undef UNPACK

  float const millirad2deg = 180.0f / ((float)M_PI * 1000.0f);
  setpoint->attitudeRate.roll = millirad2deg * values->rateRoll;
  setpoint->attitudeRate.pitch = millirad2deg * values->ratePitch;
  setpoint->attitudeRate.yaw = millirad2deg * values->rateYaw;

  quatdecompress(values->quat, (float *)&setpoint->attitudeQuaternion.q0);
  setpoint->mode.quat = modeAbs;
  setpoint->mode.roll = modeDisable;
  setpoint->mode.pitch = modeDisable;
  setpoint->mode.yaw = modeDisable;
}

/* positionDecoder
 * Set the absolute postition and orientation
 */
 struct positionPacket_s {
   float x;     // Position in m
   float y;
   float z;
   float yaw;   // Orientation in degree
 } __attribute__((packed));

static void positionDecoder(
        setpoint_t *setpoint, uint8_t type, const void *data, size_t datalen)
{
  const struct positionPacket_s *values = (positionPacket_s *)data;

  setpoint->mode.x = modeAbs;
  setpoint->mode.y = modeAbs;
  setpoint->mode.z = modeAbs;

  setpoint->position.x = values->x;
  setpoint->position.y = values->y;
  setpoint->position.z = values->z;


  setpoint->mode.yaw = modeAbs;

  setpoint->attitude.yaw = values->yaw;
}

 /* ---===== 3 - packetDecoders array =====--- */
const static packetDecoder_t packetDecoders[] = {
  [stopType]          = stopDecoder,
  [velocityWorldType] = velocityDecoder,
  [zDistanceType]     = zDistanceDecoder,
  [cppmEmuType]       = cppmEmuDecoder,
  [altHoldType]       = altHoldDecoder,
  [hoverType]         = hoverDecoder,
  [fullStateType]     = fullStateDecoder,
  [positionType]      = positionDecoder,
};

/* Decoder switch */
void crtpCommanderGenericDecodeSetpoint(setpoint_t *setpoint, crtpPacket_t *pk)
{
  static int nTypes = -1;

  //ASSERT(pk->size > 0);

  if (nTypes<0) {
    nTypes = sizeof(packetDecoders)/sizeof(packetDecoders[0]);
  }

  uint8_t type = pk->data[0];

  memset(setpoint, 0, sizeof(setpoint_t));

  if (type<nTypes && (packetDecoders[type] != NULL)) {
    packetDecoders[type](setpoint, type, ((char*)pk->data)+1, pk->size-1);
  }
}
