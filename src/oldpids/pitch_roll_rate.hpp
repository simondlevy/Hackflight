/*
  Pitch/roll angular rate PID-control algorithm for real and simulated flight
  controllers
 
  Copyright (C) 2024 Simon D. Levy
 
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, in version 3.
 
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
  */

namespace hf {

    class PitchRollRatePid {

        private:

            static constexpr float KP = 125;
            static constexpr float KI = 250;
            static constexpr float KD = 1.25;
            static constexpr float ILIMIT = 33;


        public:

            void run(
                    const float dt, 
                    const bool reset,
                    const float rollDemand,
                    const float pitchDemand,
                    const float dphi,
                    const float dtheta,
                    float & newRollDemand,
                    float & newPitchDemand)
            {

                runAxis(dt, reset, rollDemand, dphi,
                        _roll_integral, _roll_error, newRollDemand);

                runAxis(dt, reset, pitchDemand, dtheta,
                        _pitch_integral, _pitch_error, newPitchDemand);
            }

        private:

            float _roll_integral;
            float _roll_error;

            float _pitch_integral;
            float _pitch_error;

            static void runAxis(
                    const float dt,
                    const bool reset,
                    const float demand,
                    const float dangle, 
                    float & integral,
                    float & errprev,
                    float & newDemand)
            {

                const auto error = demand - dangle;

                integral = reset ? 0 :
                    Utils::fconstrain(integral + error * dt, ILIMIT);

                const auto derivative = (error - errprev) / dt;

                newDemand = KP * error + KI * integral + KD * derivative;

                errprev = error;
            }
    };

}
