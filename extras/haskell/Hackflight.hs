{--
  Hackflight core algorithm

  Copyright(C) 2021 on D.Levy

  MIT License
--}

module Hackflight where

import Receiver
import VehicleState
import Sensor
import PidController
import Demands

getVehicleState ::  [Sensor] -> VehicleState -> VehicleState

getVehicleState [] vehicleState = vehicleState 

getVehicleState sensors vehicleState = 
  addVehicleStates vehicleState' (getVehicleState (tail sensors) zeroVehicleState)
  where
    vehicleState' = (head sensors) vehicleState

hackflight :: [Sensor] -> [PidController] -> (VehicleState, Demands)

hackflight sensors pidControllers = 

    let 

        -- Get receiver demands from external C functions
        receiverDemands = Demands receiverThrottle receiverRoll receiverPitch receiverYaw

        -- Inject the receiver demands into the PID controllers
        pidControllers' = map (\p -> PidController (pidFun p) receiverDemands)
                          pidControllers

        -- Get the vehicle state by running the sensors
        vehicleState = getVehicleState sensors zeroVehicleState

        -- Map the PID update function to the pid controllers
        pidControllers'' = map (pidUpdate vehicleState) pidControllers'

        -- Sum over the list of demands to get the final demands
        newDemands = foldr addDemands receiverDemands (map pidDemands pidControllers'')

    in (vehicleState, newDemands)
