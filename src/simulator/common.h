/* 
 * Platform-independent simulator support for Hackflight
 *
 * Copyright (C) 2025 Simon D. Levy
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http:--www.gnu.org/licenses/>.
 */

#pragma once

// Structure shared between slow and fast threads
typedef struct {

    pose_t startingPose;
    float framerate;
    char path[200];
    char worldname[200];
    flightMode_t flightMode;
    demands_t setpoint;

} siminfo_t;
