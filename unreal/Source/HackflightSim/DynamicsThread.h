/*
 * HackflightSim DynamicsThread class definition
 *
 * Copyright (C) 2019 Simon D. Levy
 *
 * MIT License
 */

#pragma once

#include "Dynamics.hpp"
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
        double _startTime = 0;

        // For FPS reporting
        uint32_t _count;

        // Current actuator values from Run() method
        double _actuatorValues[100] = {}; 

        // For computing deltaT
        double   _previousTime = 0;

    public:

        // Constructor, called main thread
        FDynamicsThread(APawn * pawn, Dynamics * dynamics);

        ~FDynamicsThread(void);

        void tick(void);

        // Called by VehiclePawn::Tick() method to get actuator value for
        // animation and sound
        double actuatorValue(uint8_t index);

        uint32_t getCount(void);

        static void stopThread(FDynamicsThread ** worker);

        // FRunnable interface.

        virtual bool Init() override;

        virtual uint32_t Run() override;

        virtual void Stop() override;

}; // class FDynamicsThread
