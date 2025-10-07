class CoreTask {

    private:

        void run()
        {
            // Device-dependent
            motors_init();

            static RateSupervisor rateSupervisor;
            rateSupervisor.init(xTaskGetTickCount(), 1000, 997, 1003, 1);

            // Start with motor speeds at idle
            float motorvals[MAX_MOTOR_COUNT] = {};

            // Start with no axis demands
            demands_t demands = {};

            // Once we lose contact, we must restart
            bool lost_contact = false;

            for (uint32_t step=1; ; step++) {

                // No setpoint yet
                setpoint_t setpoint = {};

                // Wait for IMU
                _imuTask->waitDataReady();

                // Get vehicle state from estimator
                _estimatorTask->getVehicleState(&_vehicleState);

                const auto timestamp = xTaskGetTickCount();

                // If lost contact, disarm
                if (setpoint.timestamp > 0 &&
                        timestamp - setpoint.timestamp > SETPOINT_TIMEOUT_TICKS) {
                    lost_contact = true;
                    _safety->requestArming(false);
                }

                // Otherwise, run normally
                if (!lost_contact) {

                    // If armed, get demands and run mixer
                    if (_safety->isArmed()) {

                        // Get setpoint
                        _setpointTask->getSetpoint(setpoint);

                        if (Clock::rateDoExecute(CLOSED_LOOP_UPDATE_RATE, step)) {
                            runClosedLoopControl(setpoint, demands);

                            // Run closedLoopDemands through mixer to get motor speeds
                            runMixer(_mixFun, demands, motorvals);
                        }
                    }

                    // If disarmed, reset demands and motors
                    else {
                        resetDemands(demands);
                        resetMotors(motorvals);
                    }

                    // Update safety status
                    _safety->update(step, setpoint.timestamp, _vehicleState,
                            motorvals, _motorCount);

                }

                const static char * STATUS_STRINGS = {"idle", "armed", "flying", "landing"};

                DebugTask::setMessage(_debugTask,
                        "status=%s", status_strings[status]);
                        /*
                        "%05d: armed=%d hovering=%d thrust=%3.3f "
                        "m1=%3.3f m2=%3.3f m3=%3.3f m4=%3.3f",
                        step, _safety->isArmed(), setpoint.hovering, demands.thrust,
                        motorvals[0], motorvals[1],
                        motorvals[2], motorvals[3]);
                        */

                // Run motors at their current speeds (perhaps idle)
                runMotors(motorvals);

                if (!rateSupervisor.validate(timestamp)) {
                    static bool rateWarningDisplayed;
                    if (!rateWarningDisplayed) {
                        rateWarningDisplayed = true;
                    }
                }
            }
        }

        void resetMotors(float * motorvals)
        {
            for (uint8_t k=0; k<_motorCount; ++k) {
                motorvals[k] = 0;
            }
        }

        void resetDemands(demands_t & demands) 
        {
            demands.thrust = 0;
            demands.roll = 0;
            demands.pitch = 0;
            demands.yaw = 0;
        }

        void runMixer(const mixFun_t mixFun, const demands_t & demands,
                float motorvals[])
        {
            float uncapped[MAX_MOTOR_COUNT] = {};
            mixFun(demands, uncapped);

            float highestThrustFound = 0;
            for (uint8_t k=0; k<_motorCount; k++) {

                const auto thrust = uncapped[k];

                if (thrust > highestThrustFound) {
                    highestThrustFound = thrust;
                }
            }

            float reduction = 0;
            if (highestThrustFound > THRUST_MAX) {
                reduction = highestThrustFound - THRUST_MAX;
            }

            for (uint8_t k = 0; k < _motorCount; k++) {
                float thrustCappedUpper = uncapped[k] - reduction;
                motorvals[k] = thrustCappedUpper < 0 ? 0 : thrustCappedUpper / 65536;
            }
        }

        void runMotors(float * motorvals) {
            for (uint8_t id=0; id<_motorCount; ++id) {
                motors_setSpeed(id, motorvals[id]);
            }
            motors_run();
        }

        void runClosedLoopControl(
                setpoint_t & setpoint, demands_t & closedLoopDemands)
        {
            if (setpoint.hovering) {

                setpoint.demands.thrust = Num::rescale(
                        setpoint.demands.thrust, 0.2, 2.0, -1, +1);

                setpoint.demands.thrust = Num::rescale(
                        setpoint.demands.thrust, -1, +1, 0.2, 2.0);
            }

            _closedLoopControl->run(
                    1.f / CLOSED_LOOP_UPDATE_RATE,
                    setpoint.hovering,
                    _vehicleState,
                    setpoint.demands,
                    LANDING_ALTITUDE_M,
                    closedLoopDemands);
        }

        // Device-dependent ---------------------------

        void motors_init();

        void motors_setSpeed(uint32_t id, float speed);

        void motors_run();
};
