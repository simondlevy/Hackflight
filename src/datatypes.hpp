/*
   Datatype declarations

   Copyright (c) 2018 Simon D. Levy

   MIT License
   */

#pragma once

namespace hf {

    enum {
        AXIS_ROLL = 0,
        AXIS_PITCH, 
        AXIS_YAW
    };

    typedef struct {

        float throttle;
        float roll;
        float pitch;
        float yaw;

    } demands_t;

    typedef struct {

        bool armed;
        bool failsafe;

        float location[3];
        float rotation[3]; 
        float angularVel[3]; 
        float bodyAccel[3]; 
        float bodyVel[3]; 
        float inertialVel[3]; 

    } state_t;

} // namespace hf
