/*
   Copyright (c) 2022 Simon D. Levy

   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it under the
   terms of the GNU General Public License as published by the Free Software
   Foundation, either version 3 of the License, or (at your option) any later
   version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT ANY
   WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
   PARTICULAR PURPOSE. See the GNU General Public License for more details.

   You should have received a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "time.h"
#include "filter.h"

typedef struct pidAxisData_s {
    float P;
    float I;
    float D;
    float F;
    float Sum;
} pidAxisData_t;

typedef union dtermLowpass_u {
    biquadFilter_t biquadFilter;
    pt1Filter_t    pt1Filter;
    pt2Filter_t    pt2Filter;
    pt3Filter_t    pt3Filter;
} dtermLowpass_t;

typedef struct {
    pidAxisData_t  data[3];
    pt2Filter_t    dMinLowpass[3];
    pt2Filter_t    dMinRange[3];
    dtermLowpass_t dtermLowpass[3];
    dtermLowpass_t dtermLowpass2[3];
    int32_t        dynLpfPreviousQuantizedThrottle;  
    bool           feedforwardLpfInitialized;
    pt3Filter_t    feedforwardPt3[3];
    float          k_rate_p;
    float          k_rate_i;
    float          k_rate_d;
    float          k_rate_f;
    float          k_level_p;
    timeUs_t       lastDynLpfUpdateUs;
    float          previousSetpointCorrection[3];
    float          previousGyroRateDterm[3];
    float          previousSetpoint[3];
    pt1Filter_t    ptermYawLowpass;
    pt1Filter_t    windupLpf[3];

} angle_pid_t;
