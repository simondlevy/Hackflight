{--
  Hackflight core algorithm

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module Hackflight

where

import State(VehicleState)
import Demands
import Mixer(Mixer, Motors)
import PidControl(PidController, pidFun, pidState, newPidController)

type HackflightFun = Demands -> VehicleState -> PidController -> Mixer 
                     -> (Motors, PidController)

hackflightFun :: HackflightFun
hackflightFun demands vehicleState pidController mixer = 

  let controllerFun = pidFun pidController

  -- Run the PID controller to get new demands and controller state
  in let (newDemands, newControllerState) = controllerFun vehicleState
                                                          demands 
                                                          (pidState pidController)

      -- Run the mixer on the new demands to get the motors, then return them and a PID
      -- controller made from the new controller state
      in ((mixer newDemands), (newPidController controllerFun newControllerState))
