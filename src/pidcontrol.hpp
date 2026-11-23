/**
 * Copyright (C) 2026 Simon D. Levy
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <axispid.hpp>

namespace hf {

    class PidController {

        private:

            static constexpr float kAltitudeP = 2;
            static constexpr float kAltitudeI = 0.5;
            static constexpr float kAltitudeIntegralLimit = 5000;
            static constexpr float kClimbRateMax = 1;
            static constexpr float kLandingSpeedMps = 0.15;

            static constexpr float kClimbRateP = 1;
            static constexpr float kClimbRateI = 0;
            static constexpr float kClimbRateIntegralLimit = 0;

            static constexpr float kPositionP = 10;
            static constexpr float kPositionI = 1;
            static constexpr float kPositionIntegralLimit = 5000;
            static constexpr float kPositionMaxDemandDegrees = 30;

            static constexpr float kRollPitchP = 0.002;    
            static constexpr float kRollPitchI = 0.003;    
            static constexpr float kRollPitchD = 0.0005;   
            static constexpr float kRollPitchIntegralLimit = 25.0;     

            static constexpr float kYawP = 0.003;           
            static constexpr float kYawI = 0.001;          
            static constexpr float kYawD = 0.0000015;       
            static constexpr float kYawIntegralLimit = 25.0;     

            static constexpr float kAltitudeMinM = 0.2;
            static constexpr float kAltitudemaxM = 1.0;
            static constexpr float kAltitudeInitM = 0.4;
            static constexpr float kAltitudeIncMps = 0.2;
            static constexpr float kAltitudeLandingM = 0.03;
            static constexpr float kYawMaxDps = 160;     

        public:

            typedef enum {

                kControlStabilize,
                kControlAltHold,
                kControlPosHold

            } ControlLevel;

            PidController() = default;

            PidController& operator=(
                    const PidController& other) = default;

            static auto Run(
                    const PidController & pc,
                    const bool airborne,
                    const float dt,
                    const Mode mode,
                    const VehicleState & state,
                    const Setpoint & setpoint_in,
                    const ControlLevel
                    control_level=kControlPosHold) -> PidController
            {
                // Altitude hold ---------------------------------------------

                const auto  altitude_target =
                    pc.altitude_target_ == 0 ? kAltitudeInitM :
                    pc.altitude_target_;

                const auto new_altitude_target = Num::ConstrainFloat(
                        altitude_target +
                        setpoint_in.thrust * kAltitudeIncMps * dt,
                        kAltitudeMinM, kAltitudemaxM);

                const auto hovering =
                    mode == kModeHover || mode == kModeAutonomous;

                const auto altitude_pid =
                    AxisPid::Run(
                            pc.altitude_pid_,
                            kAltitudeP,
                            kAltitudeI,
                            kAltitudeIntegralLimit,
                            hovering,
                            dt,
                            new_altitude_target,
                            state.z,
                            -kClimbRateMax,
                            +kClimbRateMax,
                            0);

                const auto climbrate_target =
                    hovering ? altitude_pid.output : -kLandingSpeedMps;

                const auto climbrate_pid =
                    AxisPid::Run(
                            pc.climbrate_pid_,
                            kClimbRateP,
                            kClimbRateI,
                            kClimbRateIntegralLimit,
                            hovering || (state.z > kAltitudeLandingM),
                            dt,
                            climbrate_target,
                            state.dz,
                            0,
                            1,
                            0.5);

                // Position hold ---------------------------------------------

                // Rotate world-coordinate velocities into body coordinates
                const auto dxw = state.dx;
                const auto dyw = state.dy;
                const auto psi = Num::kDeg2Rad * state.psi;
                const auto cospsi = cos(psi);
                const auto sinpsi = sin(psi);
                const auto dxb =  dxw * cospsi + dyw * sinpsi;
                const auto dyb = -dxw * sinpsi + dyw * cospsi;       

                const auto position_y_pid =
                    AxisPid::Run(
                            pc.position_y_pid_,
                            kPositionP,
                            kPositionI,
                            kPositionIntegralLimit,
                            airborne,
                            dt,
                            setpoint_in.roll,
                            dyb,
                            -kPositionMaxDemandDegrees,
                            +kPositionMaxDemandDegrees,
                            0);

                const auto position_x_pid =
                    AxisPid::Run(
                            pc.position_x_pid_,
                            kPositionP,
                            kPositionI,
                            kPositionIntegralLimit,
                            airborne,
                            dt,
                            setpoint_in.pitch,
                            dxb,
                            -kPositionMaxDemandDegrees,
                            +kPositionMaxDemandDegrees,
                            0);

                const auto hold_position = control_level == kControlPosHold;

                const auto roll_demand = hold_position ? position_y_pid.output :
                    BypassPositionControl(setpoint_in.roll,
                            kPositionMaxDemandDegrees);

                const auto pitch_demand = hold_position ? position_x_pid.output :
                    BypassPositionControl(setpoint_in.pitch,
                            kPositionMaxDemandDegrees);

                // Stabilize  ------------------------------------------------

                const auto roll_pid = AxisPid::Run(
                        pc.roll_pid_,
                        kRollPitchP,    
                        kRollPitchI,    
                        kRollPitchD,   
                        kRollPitchIntegralLimit,     
                        dt,
                        airborne,
                        roll_demand,
                        state.phi,
                        -state.dphi);

                const auto pitch_pid = AxisPid::Run(
                        pc.pitch_pid_,
                        kRollPitchP,    
                        kRollPitchI,    
                        kRollPitchD,   
                        kRollPitchIntegralLimit,     
                        dt,
                        airborne,
                        pitch_demand,
                        state.theta,
                        -state.dtheta);

                const auto yaw_pid = AxisPid::Run(
                        pc.yaw_pid_,
                        kYawP, 
                        kYawI,
                        kYawD,
                        kYawIntegralLimit,
                        dt,
                        airborne,
                        setpoint_in.yaw * kYawMaxDps,
                        state.dpsi);

                const auto setpoint_out = Setpoint(

                        control_level > kControlStabilize ?
                        climbrate_pid.output :
                        (setpoint_in.thrust + 1) / 2, // [-1,+1] => [0,1]

                        roll_pid.output,
                        pitch_pid.output,
                        yaw_pid.output);

                return PidController(
                        new_altitude_target,
                        altitude_pid,
                        climbrate_pid,
                        pc.position_x_pid_,
                        pc.position_y_pid_,
                        roll_pid,
                        pitch_pid,
                        yaw_pid,
                        setpoint_out);
            }

            static auto GetSetpoint(const PidController & pc) -> Setpoint
            {
                return pc.setpoint_;
            }

            static auto GetYawPid(const PidController & pc) -> AxisPid
            {
                return pc.yaw_pid_;
            }

            static auto GetPitchPid(const PidController & pc) -> AxisPid
            {
                return pc.pitch_pid_;
            }

        private:

            Setpoint setpoint_;

            float altitude_target_;

            PidController(
                    const float altitude_target,
                    const AxisPid & altitude_pid,
                    const AxisPid & climbrate_pid,
                    const AxisPid & position_x_pid,
                    const AxisPid & position_y_pid,
                    const AxisPid & roll_pid,
                    const AxisPid & pitch_pid,
                    const AxisPid & yaw_pid,
                    const Setpoint & setpoint)
                :
                    setpoint_(setpoint),
                    altitude_target_(altitude_target),
                    altitude_pid_(altitude_pid),
                    climbrate_pid_(climbrate_pid),
                    position_x_pid_(position_x_pid),
                    position_y_pid_(position_y_pid),
                    roll_pid_(roll_pid),
                    pitch_pid_(pitch_pid),
                    yaw_pid_(yaw_pid) {}


            static auto BypassPositionControl(
                    const float in, const float maxval) -> float
            {
                return in * maxval;
            }

            static auto GetPidValues(
                    const AxisPid & pid) -> std::vector<float> 
            {
                return {
                    pid.target,
                    pid.actual,
                    pid.error,
                    pid.integral,
                    pid.derivative,
                    pid.output
                };
            }

        public:

            AxisPid altitude_pid_;
            AxisPid climbrate_pid_;

            AxisPid position_x_pid_;
            AxisPid position_y_pid_;

            AxisPid roll_pid_;
            AxisPid pitch_pid_;

            AxisPid yaw_pid_;
    };
}
