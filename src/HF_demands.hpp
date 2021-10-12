/*
   Declarations for axis demands

   Copyright (c) 2021 Simon D. Levy

   MIT License
   */

#pragma once

namespace hf {

    typedef struct {
        float throttle;
        float roll;
        float pitch;
        float yaw;
    } demands_t;

    enum {
        DEMANDS_THROTTLE,
        DEMANDS_ROLL,
        DEMANDS_PITCH,
        DEMANDS_YAW,
    };

} // namespace hf
