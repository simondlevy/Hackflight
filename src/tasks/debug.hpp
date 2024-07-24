#pragma once

#include <hackflight.hpp>
#include <timer.hpp>

class DebugTask {

    public:

        typedef enum {

            NONE,
            RADIO,
            DEMANDS,
            ANGLES,
            DANGLES,
            MOTORS

        } mode_e;

        void run(
                const uint32_t usec_curr, 
                const float freq_hz, 
                mode_e mode,
                const channels_t & channels,
                const demands_t & demands,
                const state_t & state,
                const axis3_t gyro,
                const axis3_t accel,
                const quad_motors_t & motors)
        {
            if (_timer.isReady(usec_curr, freq_hz)) {

                switch (mode) {

                    case RADIO:
                        debugRadioData(channels);     
                        break;
                    case DEMANDS:
                        debugDemands(demands);
                        break;
                    case ANGLES:
                        debugAngles(state);  
                        break;
                    case DANGLES:
                        debugDangles(state);  
                        break;
                    case MOTORS:
                        debugMotorCommands(motors); 
                        break;
                    default:
                        break;
                }
            }
        }

    private:

        Timer _timer;

        void debugRadioData(const channels_t & channels) 
        {
            Serial.printf("ch1:%ld ch2:%ld ch3:%ld ch4:%ld ch5:%ld ch6:%ld\n", 
                    channels.c1, channels.c2, channels.c3,
                    channels.c4, channels.c5, channels.c6);
        }

        void debugDemands(const demands_t & demands)
        {
            Serial.printf("thrust:%3.3f roll:%+3.3f pitch:%+3.3f yaw:%+3.3f\n",
                    demands.thrust, demands.roll, demands.pitch, demands.yaw);
        }

        void debugAngles(const state_t & state)
        {
            Serial.printf("roll\t:%+03d pitch:%+03d\tyaw:%+03d\n", 
                    (int)state.phi, (int)state.theta, (int)state.psi);
        }

        void debugDangles(const state_t & state)
        {
            Serial.printf("droll  :%+03d\tdpitch:%+03d\tdyaw:%+03d\n", 
                    (int)state.dphi, (int)state.dtheta, (int)state.dpsi);
        }

        void debugMotorCommands(const quad_motors_t & motors) 
        {
            Serial.printf("m1:%f m2:%f m3:%f m4:%f\n",
                    motors.m1, motors.m2, motors.m3, motors.m4);
        }
};
