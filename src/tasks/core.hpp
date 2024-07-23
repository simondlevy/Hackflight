#pragma once

#include <datatypes.h>
#include <mixers.hpp>
#include <utils.hpp>

#include <pids/altitude.hpp>
#include <pids/climb_rate.hpp>
#include <pids/pitch_roll_angle.hpp>
#include <pids/pitch_roll_rate.hpp>
#include <pids/position.hpp>
#include <pids/yaw_angle.hpp>
#include <pids/yaw_rate.hpp>

class CoreTask {

    public:

        void init(
                const float pitch_roll_angle_kp, 
                const float pitch_roll_rate_kp, 
                const float pitch_roll_rate_kd,
                const float yaw_rate_kp, 
                const float tbase, 
                const float tscale, 
                const float tmin,
                const float dt)
        {
                _pitch_roll_angle_kp = pitch_roll_angle_kp; 
                _pitch_roll_rate_kp = pitch_roll_rate_kp; 
                _pitch_roll_rate_kd =pitch_roll_rate_kd;
                _yaw_rate_kp = yaw_rate_kp; 
                _tbase = tbase; 
                _tscale = tscale; 
                _tmin =tmin;
                _dt = dt;
         }

         void run(
                 const state_t & state, 
                 const demands_t & stickDemands, 
                 quad_motors_t & motors)
        {
            static flyingStatus_e _status;

            static float _altitude_target;

            _altitude_target =
                _status == STATUS_FLYING ? 
                _altitude_target + THROTTLE_SCALE * stickDemands.thrust :
                _status == STATUS_LANDED ?
                INITIAL_ALTITUDE_TARGET :
                _altitude_target;

            _status = 

                _status == STATUS_TAKING_OFF  && state.z > ZGROUND ?  
                STATUS_FLYING :

                _status == STATUS_FLYING && state.z <= ZGROUND ?  
                STATUS_LANDED :

                _status == STATUS_LANDED && 
                stickDemands.thrust > THROTTLE_ZERO ? 
                STATUS_TAKING_OFF :

                _status;

            const auto landed = _status == STATUS_LANDED;

            demands_t demands = { 
                stickDemands.thrust,
                stickDemands.roll,
                stickDemands.pitch,
                stickDemands.yaw
            };

            _positionController.run(state, _dt, demands);

            _pitchRollAngleController.run(
                    _pitch_roll_angle_kp, state, _dt, demands);

            _pitchRollRateController.run(_pitch_roll_rate_kp,
                    _pitch_roll_rate_kd, state, _dt, demands);

            _altitudeController.run(state, _dt, _altitude_target, demands);

            _yawAngleController.run(state, _dt, demands);

            _yawRateController.run(_yaw_rate_kp, state, _dt, demands);

            _climbRateController.run(state, _dt, _tbase, _tscale, _tmin,
                    !landed, demands);

            // Run mixer to convert demands to motor spins
            Mixer::runCF(demands, motors);
        }

    private:

         typedef enum {

             STATUS_LANDED,
             STATUS_TAKING_OFF,
             STATUS_FLYING

         } flyingStatus_e;

         // We consider throttle inputs above this below this value to be
         // positive for takeoff
         static constexpr float THROTTLE_ZERO = 0.05;

         static constexpr float THROTTLE_SCALE = 0.005;

         // We consider altitudes below this value to be the ground
         static constexpr float ZGROUND = 0.05;

         static constexpr float INITIAL_ALTITUDE_TARGET = 0.2;

         // PID constants set by each platform
         float _pitch_roll_angle_kp; 
         float _pitch_roll_rate_kp; 
         float _pitch_roll_rate_kd;
         float _yaw_rate_kp; 
         float _tbase; 
         float _tscale; 
         float _tmin;
         float _dt;

         PositionController _positionController;
         PitchRollAngleController _pitchRollAngleController;
         PitchRollRateController _pitchRollRateController;
         AltitudeController _altitudeController;
         YawAngleController _yawAngleController;
         YawRateController _yawRateController;
         ClimbRateController _climbRateController;

};
