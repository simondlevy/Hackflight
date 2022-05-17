/*
 * HackflightSim DynamicsThread class definition
 *
 * Copyright (C) 2019 Simon D. Levy
 *
 * MIT License
 */

#pragma once

#include "Utils.hpp"
#include "GameInput.hpp"

#include "Runtime/Core/Public/HAL/Runnable.h"

class FDynamicsThread : public FRunnable {

    private:

        // Thread and guards
        FRunnableThread * _thread = NULL;
        bool _ready = false;
        bool _running = false;

        // Start-time offset so timing begins at zero
        float _startTime = 0;

        // Current actuator values from Run() method
        float _motorValues[4] = {}; 

    public:

        // Constructor, called main thread
        FDynamicsThread(APawn * pawn);

        ~FDynamicsThread(void);

        void tick(void);

        void setAgl(float agl);

        // Called by VehiclePawn::Tick() method to get actuator value for
        // animation and sound
        float getActuatorValue(uint8_t index);

        static void stopThread(FDynamicsThread ** worker);

        // FRunnable interface.

        virtual bool Init() override;

        virtual uint32_t Run() override;

        virtual void Stop() override;

        void getPose(float & x, float & y, float & z, float & phi, float & theta, float & psi);

}; // class FDynamicsThread
