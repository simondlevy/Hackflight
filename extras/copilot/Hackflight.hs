{--
  Haskell Copilot support for Hackflight

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}
{-# LANGUAGE DataKinds        #-}

module Main where

import Language.Copilot
import Copilot.Compile.C99

import Demands
import VehicleState
import Gyrometer
-- import Quaternion
import EulerAngles

receiverThrottle :: Stream Double
receiverThrottle  = extern "receiverThrottle" Nothing

spec = do

  let sensors = [eulerModifyState, gyroModifyState]

  -- Get the vehicle state by running the sensors
  let vehicleState = foldr addStates initialVehicleState sensors

  trigger "runMotors" true [arg receiverThrottle]

-- Compile the spec
main = reify spec >>= compile "hackflight"
