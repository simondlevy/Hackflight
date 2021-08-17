{--
  Haskell Copilot support for Hackflight

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}
{-# LANGUAGE DataKinds        #-}

module Main where

import Language.Copilot
import Copilot.Compile.C99

--import Demands
import VehicleState
import Altimeter
import Utils(compose)
--import Gyrometer
-- import Quaternion
--import EulerAngles

receiverThrottle :: Stream Double
receiverThrottle  = extern "receiverThrottle" Nothing

receiverRoll :: Stream Double
receiverRoll  = extern "receiverRoll" Nothing

receiverPitch :: Stream Double
receiverPitch  = extern "receiverPitch" Nothing

receiverYaw :: Stream Double
receiverYaw  = extern "receiverYaw" Nothing

spec = do

  let sensors = [altimeter]

  -- Get the vehicle state by running the sensors
  let vehicleState = compose sensors initialVehicleState

  let motor = if receiverThrottle < 0 then 0 else receiverThrottle

  let m1 = motor
  let m2 = motor
  let m3 = motor
  let m4 = motor

  trigger "runMotors" true [arg m1, arg m2, arg m3, arg m4]

  trigger "showVehicleState" true [  arg (z vehicleState)
                                   , arg (dz vehicleState)
                                  ]

-- Compile the spec
main = reify spec >>= compile "hackflight"
