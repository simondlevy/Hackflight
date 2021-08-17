{--
  Haskell Copilot support for pass-through "sensor"

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

{-# LANGUAGE DataKinds        #-}

module SimSensor

where

import Language.Copilot

import VehicleState

simSensorZ :: Stream Double
simSensorZ = extern "simSensorZ" Nothing

simSensorDz :: Stream Double
simSensorDz = extern "simSensorDz" Nothing

simSensorModifyState :: VehicleState

simSensorModifyState = VehicleState simSensorZ simSensorDz
