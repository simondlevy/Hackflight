/*
   Declarations for motor control

   Copyright (c) 2021 Simon D. Levy

   MIT License
   */

#pragma once

typedef struct {
    bool running;
    float values[4];
} motors_t;
