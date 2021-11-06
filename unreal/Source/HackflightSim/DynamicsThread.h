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

        // For guarding thread
        bool _ready = false;

        FRunnableThread * _thread = NULL;

        bool _running = false;

        // Start-time offset so timing begins at zero
        double _startTime = 0;

        // For FPS reporting
        uint32_t _count;

        // Current actuator values from getActuators() method
        double _actuatorValues[100] = {}; 

        // For computing deltaT
        double   _previousTime = 0;

        /**
         * Flight-control method running repeatedly on its own thread.  
         * @param time current time in seconds (input)
         * @param values actuator values returned by your controller (output)
         *
         */
        void getActuators(const double time, double * values);

        uint8_t _actuatorCount = 0;

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
