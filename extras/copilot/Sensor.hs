{--
  Haskell Copilot support for sensors

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module Sensor

where

import Language.Copilot

import VehicleState

data Sensor = Gyrometer { gyroX :: Stream Double
                        , gyroY :: Stream Double
                        , gyroZ :: Stream Double } 


modifyState :: Sensor -> VehicleState -> VehicleState

modifyState (Gyrometer gx gy gz) vehicleState = 

  VehicleState (x vehicleState)
               (dx vehicleState)
               (y vehicleState)
               (dy vehicleState)
               (z vehicleState)
               (dz vehicleState)
               (phi vehicleState)
               gx
               (theta vehicleState)
               gy
               (psi vehicleState)
               gz
