/*
 * HackflightSim FlightManager class definition
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

class FFlightManager : public FRunnable {

    private:

        // Joystick (RC transmitter, game controller) or keypad
        GameInput * _gameInput = NULL;
        double _joyvals[4] = {};

        // For guarding thread
        bool _ready = false;

        // Helpers
        void getReceiverDemands(void);
        void getGyrometer(void);
        void getQuaternion(void);
        void getOpticalFlow(void);

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

    protected:

        uint8_t _actuatorCount = 0;

        Dynamics * _dynamics = NULL;

        // Constructor, called main thread
        FFlightManager(Dynamics * dynamics);

    public:

        FFlightManager(APawn * pawn, Dynamics * dynamics);

        ~FFlightManager(void);

        void tick(void);

        // Called by VehiclePawn::Tick() method to get actuator value for
        // animation and sound
        double actuatorValue(uint8_t index);

        uint32_t getCount(void);

        static void stopThread(FFlightManager ** worker);

        // FRunnable interface.

        virtual bool Init() override;

        virtual uint32_t Run() override;

        virtual void Stop() override;

}; // class FFlightManager
