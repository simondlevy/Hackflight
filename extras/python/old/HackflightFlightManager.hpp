/*
   MulticopterSim FlightManager class using Hackflight

   Just spins propellers

   Copyright(C) 2019 Simon D.Levy

   MIT License
*/

#include "../MainModule/FlightManager.hpp"
#include "../MainModule/Dynamics.hpp"
#include "../MainModule/Transforms.hpp"

#include <hackflight.hpp>

// PID controllers
#include <pidcontrollers/level.hpp>
#include <pidcontrollers/yaw.hpp>
#include <pidcontrollers/rate.hpp>
// #include <pidcontrollers/althold.hpp>

#include <rft_closedloops/passthru.hpp>


#include "SimReceiver.hpp"
#include "SimBoard.hpp"
#include "SimMotor.hpp"
#include "SimSensors.hpp"

class FHackflightFlightManager : public FFlightManager {

    private:

        // PID tuning

		// Rate
		hf::RatePid ratePid = hf::RatePid(.01, .01, .01);	

   		// Yaw 
		hf::YawPid yawPid = hf::YawPid(.025, .01);

        // Level
        hf::LevelPid levelPid = hf::LevelPid(1.0);

        rft::PassthruController passthru;

        /*
        // Alt-hold
        hf::AltitudeHoldPid althold = hf::AltitudeHoldPid(
                10.00f, // altHoldPosP
                1.00f,  // altHoldVelP
                0.01f,  // altHoldVelI
                0.10f); // altHoldVelD
                */

        // Flight-controller board
        SimBoard _board;

        // "Receiver" (joystick/gamepad)
        SimReceiver * _receiver = NULL;

        // Sensors"
        SimSensors* _sensors = NULL;

        // Helps us access individual motors
        SimMotor* _motors = NULL;

        // Main firmware
        hf::Hackflight * _hackflight = NULL;

    public:

        // Constructor
        FHackflightFlightManager(APawn * pawn, hf::Mixer * mixer, SimMotor * motors, Dynamics * dynamics, 
                bool pidsEnabled=true) 
            : FFlightManager(dynamics) 
        {
            _motors = motors;

            // Pass PlayerController to receiver constructor in case we have no joystick / game-controller
            _receiver = new SimReceiver(UGameplayStatics::GetPlayerController(pawn->GetWorld(), 0));

            _hackflight = new hf::Hackflight(&_board, _receiver, mixer);

            // Add simulated sensor suite
            _sensors = new SimSensors(_dynamics);
            _hackflight->addSensor(_sensors);

            if (pidsEnabled) {

                // Add altitude-hold PID controller in switch position 1 or greater
                //_hackflight->addClosedLoopController(&althold, 1);

                // Add yaw and level PID controllers for all aux switch positions
                _hackflight->addClosedLoopController(&levelPid);
                _hackflight->addClosedLoopController(&ratePid);
                _hackflight->addClosedLoopController(&yawPid);
            }

            // _hackflight->addClosedLoopController(&passthru);

            // Start Hackflight firmware, indicating already armed
            _hackflight->begin(true);
        }

        virtual ~FHackflightFlightManager(void)
        {
            delete _hackflight;
        }

        virtual void getMotors(const double time, double * motorvals) override
        {
            // Zero on success, nonzero otherwise
            uint16_t joystickError = _receiver->update();

            double angularVel[3] = {
                _dynamics->x(Dynamics::STATE_PHI_DOT),
                _dynamics->x(Dynamics::STATE_THETA_DOT),
                _dynamics->x(Dynamics::STATE_PSI_DOT) 
            };

            double eulerAngles[3] = {
                _dynamics->x(Dynamics::STATE_PHI),
                _dynamics->x(Dynamics::STATE_THETA),
                _dynamics->x(Dynamics::STATE_PSI) 
            };

            double quaternion[4] = {};
            Transforms::eulerToQuaternion(eulerAngles, quaternion);

            // If joystick missing or bad, use keypad
            if (joystickError) {
            }

            _hackflight->update();

            _board.set(time);

            // _imu.set(quaternion, angularVel);

            //  Get motor values
            for (uint8_t i=0; i < _nmotors; ++i) {
                motorvals[i] = _motors->getValue(i);
            }
        }

        void tick(void)
        {
            _receiver->tick();
        }

}; // HackflightFlightManager
