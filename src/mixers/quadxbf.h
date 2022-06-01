#pragma once

/*
 * Mixer values for quad-X frames using Betaflight motor layout:
 *
 *    4cw   2ccw
 *       \ /
 *        ^
 *       / \
 *    3ccw  1cw
 *
 * Copyright (C) 2022 Simon D. Levy
 *
 */

#include "datatypes.h"

// Quad X configuration with Betaflight numbering
static axes_t mixerQuadXBF[] = {
    //  rol   pit    yaw
    { -1.0f, +1.0f, -1.0f },          // REAR_R
    { -1.0f, -1.0f, +1.0f },          // FRONT_R
    { +1.0f, +1.0f, +1.0f },          // REAR_L
    { +1.0f, -1.0f, -1.0f },          // FRONT_L
};
