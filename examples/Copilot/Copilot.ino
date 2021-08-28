/*
   Hackflight using Haskell Copilot

   Copyright(C) 2021 Simon D.Levy

   MIT License
*/

#include "copilot_hackflight.h"

// XXX eventually most of this functionality should be in Haskell
#include "HF_full.hpp"
#include "hf_receivers/arduino/dsmx/dsmx_serial1.hpp"


// Used by Copilot ---------------------------------

float copilot_time = 0;

float copilot_receiverThrottle = 0;
float copilot_receiverRoll = 0;
float copilot_receiverPitch = 0;
float copilot_receiverYaw = 0;

bool copilot_receiverLostSignal = false;
bool copilot_receiverReady = true;
bool copilot_receiverInArmedState = true;
bool copilot_receiverInactive = false;

float copilot_gyrometerX = 0;
float copilot_gyrometerY = 0;
float copilot_gyrometerZ = 0;

float copilot_quaternionW = 0;
float copilot_quaternionX = 0;
float copilot_quaternionY = 0;
float copilot_quaternionZ = 0;

// Called by Copilot
void copilot_runMotors(float m1, float m2, float m3, float m4)
{
}

void setup(void)
{
}

void loop(void)
{
    step();
}
