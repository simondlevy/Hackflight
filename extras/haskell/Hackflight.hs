{--
  Hackflight core algorithm

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module Hackflight

where

import State(Time, VehicleState)
import Demands
import Mixer(Mixer, Motors)
import PidControl(PidFun, PidState)

type HackflightFun = Demands -> Time -> VehicleState -> (PidFun, PidState) -> Mixer 
                     -> (Motors, (PidFun, PidState))

hackflightFun :: HackflightFun
hackflightFun demands time vstate controller mixer = 

  -- Run the PID controller to get new demands and controller state
  let (newDemands, newControllerState) = (fst controller) time vstate demands (snd controller)

  -- Run the mixer on the new demands to get the motors, then return them and the new
  -- controller state
  in ((mixer newDemands), ((fst controller), newControllerState))
