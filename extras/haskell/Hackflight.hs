{--
  Hackflight core algorithm

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module Hackflight(HackflightFun, hackflightFun)

where

import State(VehicleState)
import Demands
import Mixer(Mixer, Motors)
import PidControl(PidController, pidFun, pidState, newPidController)

type HackflightFun = Demands -> VehicleState -> [PidController] -> Mixer 
                     -> (Motors, [PidController])

hackflightFun :: HackflightFun

-- Base case: apply the mixer to the demands to get the motor values
hackflightFun demands _vehicleState [] mixer = 
    (mixer demands, [])

hackflightFun demands vehicleState pidControllers mixer = 

    let pidController = pidControllers!!0
      
        controllerFun = pidFun pidController

        -- Run the PID controller to get new demands and controller state
        (newDemands, newControllerState) = controllerFun vehicleState
                                                         demands 
                                                         (pidState pidController)

    -- Run the mixer on the new demands to get the motors, then return them and a PID
    -- controller made from the new controller state
    in ((mixer newDemands), [(newPidController controllerFun newControllerState)])
