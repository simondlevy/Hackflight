{--
  Haskell Copilot support for sensors

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

{-# LANGUAGE DataKinds        #-}

module Sensors

where

import Language.Copilot

import VehicleState

data Sensor = Gyrometer { gyroX :: Stream Double
                        , gyroY :: Stream Double
                        , gyroZ :: Stream Double } 

type SensorFun = Sensor -> VehicleState -> VehicleState

gyroModifyState :: SensorFun

gyroModifyState (Gyrometer gx gy gz) vehicleState = 

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

gyrometerValues :: Stream (Array 3 Double)
gyrometerValues  = extern "gyrometerValues" Nothing
