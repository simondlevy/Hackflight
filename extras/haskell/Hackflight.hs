{--
  Hackflight core algorithm

  Copyright(C) 2021 on D.Levy

  MIT License
--}

module Hackflight where

import Receiver
import State
import Sensor
import PidController
import Demands
import Safety
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
        state = compose sensors zeroState

        -- Map the PID update function to the pid controllers
        pidControllers'' = map (pidUpdate state) pidControllers'

        -- Sum over the list of demands to get the final demands
        demands = foldr addDemands receiverDemands (map pidDemands pidControllers'')

        -- Map throttle from [-1,+1] to [0,1]
        demands' = Demands (((throttle demands) + 1) / 2)
                           (roll demands)
                           (pitch demands)
                           (yaw demands)

    in demands'
