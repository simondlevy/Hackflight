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
