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

simSensorModifyState :: VehicleState

simSensorModifyState = VehicleState 0 0 0 0 simSensorZ 0 0 0 0 0 0 0
