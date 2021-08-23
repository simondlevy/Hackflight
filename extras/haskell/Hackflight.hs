{--
  Hackflight core algorithm

  Copyright(C) 2021 on D.Levy

  MIT License
--}

module Hackflight where

import Receiver
import VehicleState(zeroVehicleState)
import Sensor
import PidController
import Demands
import Utils(compose)

hackflight :: [Sensor] -> [PidController] -> Demands

hackflight sensors pidControllers = 

    let 

        -- Get receiver demands from external C functions
        receiverDemands = Demands receiverThrottle receiverRoll receiverPitch receiverYaw

        -- Inject the receiver demands into the PID controllers
        pidControllers' = map (\p -> PidController (pidFun p) receiverDemands)
                          pidControllers

        -- Get the vehicle state by running the sensors
        vehicleState = compose sensors zeroVehicleState

        -- Map the PID update function to the pid controllers
        pidControllers'' = map (pidUpdate vehicleState) pidControllers'

        -- Sum over the list of demands to get the final demands
        newDemands = foldr addDemands receiverDemands (map pidDemands pidControllers'')

    in newDemands
