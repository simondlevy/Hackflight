/*
 * Abstract, threaded flight-management class for MulticopterSim
 *
 * Copyright (C) 2019 Simon D. Levy
 *
 * MIT License
 */

#pragma once

#include "Dynamics.hpp"
#include "Utils.hpp"

#include "Runtime/Core/Public/HAL/Runnable.h"

class FFlightManager : public FRunnable {

    private:

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
         * Override this method to implement your own flight controller.
         *
         * @param time current time in seconds (input)
         * @param values actuator values returned by your controller (output)
         *
         */
        virtual void getActuators(const double time, double * values)  = 0;

    protected:

        uint8_t _actuatorCount = 0;

        Dynamics * _dynamics = NULL;

        // Constructor, called main thread
        FFlightManager(Dynamics * dynamics) 
        {
            _thread = FRunnableThread::Create(this, TEXT("FThreadedManage"), 0, TPri_BelowNormal); 
            _startTime = FPlatformTime::Seconds();
            _count = 0;

            // Constant
            _actuatorCount = dynamics->actuatorCount();

            _dynamics = dynamics;

            // For periodic update
            _previousTime = 0;
        }

        uint32_t getFps(void)
        {
            return (uint32_t)(_count/(FPlatformTime::Seconds()-_startTime));
        }


    public:

        ~FFlightManager(void)
        {
            delete _thread;
        }

        // Called by VehiclePawn::Tick() method to get actuator value for
        // animation and sound
        double actuatorValue(uint8_t index)
        {
            return _actuatorValues[index];
        }

        uint32_t getCount(void)
        {
            return _count;
        }

        static void stopThread(FFlightManager ** worker)
        {
            if (*worker) {
                (*worker)->Stop();
                delete *worker;
            }

            *worker = NULL;
        }

        // FRunnable interface.

        virtual bool Init() override
        {
            _running = false;

			return FRunnable::Init();
        }

        virtual uint32_t Run() override
        {
            // Initial wait before starting
            FPlatformProcess::Sleep(0.5);

            _running = true;

            while (_running) {

                // Get a high-fidelity current time value from the OS
                double currentTime = FPlatformTime::Seconds() - _startTime;

                // Update dynamics
                _dynamics->update(_actuatorValues, currentTime - _previousTime);

                // PID controller: update the flight manager (e.g.,
                // HackflightManager) with the dynamics state, getting back the
                // actuator values
                this->getActuators(currentTime, _actuatorValues);

                // Track previous time for deltaT
                _previousTime = currentTime;

                // Increment count for FPS reporting
                _count++;
            }

            return 0;
        }

        virtual void Stop() override
        {
            _running = false;

            // Final wait after stopping
            FPlatformProcess::Sleep(0.03);

            FRunnable::Stop();
        }

}; // class FFlightManager
