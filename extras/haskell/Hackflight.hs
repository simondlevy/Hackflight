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
import PidControl(PidController, pidFun, pidState, newPidController)

type HackflightFun = Demands -> Time -> VehicleState -> PidController -> Mixer 
                     -> (Motors, PidController)

hackflightFun :: HackflightFun
hackflightFun demands time vehicleState pidController mixer = 

  -- Run the PID controller to get new demands and controller state
  let (newDemands, newControllerState) = (pidFun pidController) time
                                                                vehicleState
                                                                demands 
                                                                (pidState pidController)

  -- Run the mixer on the new demands to get the motors, then return them and the new
  -- controller state
  in ((mixer newDemands), (newPidController (pidFun pidController) newControllerState))
