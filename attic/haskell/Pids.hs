{--
  Copyright (C) 2025 Simon D. Levy
 
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, in version 3.
 
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
--} 

{-# LANGUAGE DataKinds #-}
{-# LANGUAGE RebindableSyntax #-}

module Pids where

import Language.Copilot
import Copilot.Compile.C99

import Num

-- PID controllers
import AltitudeController
import ClimbRateController
import YawAngleController
import YawRateController
import PositionController
import PitchRollAngleController
import PitchRollRateController

-- Constants -----------------------------------------------------------------

spec = do

    let climbrate = altitudeController

    let thrust = climbRateController climbrate 

    let yaw_rate = yawAngleController

    let yaw' = yawRateController yaw_rate

    let (roll', pitch') = positionController

    let (roll'', pitch'') = pitchRollAngleController (roll', pitch')

    let (roll''', pitch''') = pitchRollRateController (roll'', pitch'')

    trigger "setDemands" true 
                         [arg thrust, arg roll''', arg pitch''', arg yaw']

-- Compile the spec
main = reify spec >>= 
  compileWith (CSettings "copilot_step_core" ".") "copilot_core"
