/*
   This file is part of Hackflight.

   Hackflight is free software: you can redistribute it and/or modify it under the
   terms of the GNU General Public License as published by the Free Software
   Foundation, either version 3 of the License, or (at your option) any later
   version.

   Hackflight is distributed in the hope that it will be useful, but WITHOUT ANY
   WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
   PARTICULAR PURPOSE. See the GNU General Public License for more details.

   You should have received a copy of the GNU General Public License along with
   Hackflight. If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

class Arming {

    private:

        bool m_accDoneCalibrating;
        bool m_angleOkay;
        bool m_gotFailsafe;
        bool m_gyroDoneCalibrating;
        bool m_haveSignal;
        bool m_isArmed;
        bool m_switchOkay;
        bool m_throttleIsDown;


    public:

        bool ready(void)
        {
            return 
                m_accDoneCalibrating &&
                m_angleOkay &&
                !m_gotFailsafe &&
                m_haveSignal &&
                m_gyroDoneCalibrating &&
                m_switchOkay &&
                m_throttleIsDown;
        }

        void updateFromImu(const bool imuIsLevel, const bool gyroIsCalibrating)
        {
            m_angleOkay = imuIsLevel;
            m_gyroDoneCalibrating = !gyroIsCalibrating;
            m_accDoneCalibrating = true; // XXX
        }

}; // class Arming
