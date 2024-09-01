/*
  X/Y position PID control algorithm for real and simulated flight controllers
 
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

/*
run kp ki reset dt ilimit target actual integ = (demand, integ') where

  error = target - actual

  demand = (-(kp * error + ki * integ))

  integ' = if reset 
           then 0
           else constrain (integ + error * dt) (-ilimit) (ilimit)

{--
  Position controller converts meters per second to  degrees.

  Demands are input as normalized interval [-1,+1] and output as angles in 
  degrees:

   roll:  input left positive => output negative

   pitch: input forward positive => output negative
--}

positionPid :: ClosedLoopController

positionPid reset dt state demands = demands'  where

  kp = 25
  ki = 1
  ilimit = 5000
    
  (rollDemand, rollInteg) = 
    run kp ki reset dt ilimit (roll demands) (dy state) rollInteg'

  rollInteg' = [0] ++ rollInteg

  (pitchDemand, pitchInteg) = 
    run kp ki reset dt ilimit (pitch demands) (dx state) pitchInteg'

  pitchInteg' = [0] ++ pitchInteg

  demands' = Demands (thrust demands) rollDemand pitchDemand (yaw demands)
  */
