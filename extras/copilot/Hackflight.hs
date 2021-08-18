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

import PidControllers
import Receiver
import VehicleState
import Altimeter
import Utils(compose)
import Gyrometer
import Euler

import FullPidController

spec = do

  let sensors = [euler, gyrometer, altimeter]

  -- Get the vehicle state by running the sensors
  let vehicleState = compose sensors initialVehicleState

  let motor = if receiverThrottle < 0 then 0 else receiverThrottle

  let m1 = motor
  let m2 = motor
  let m3 = motor
  let m4 = motor

  trigger "runMotors" true [arg m1, arg m2, arg m3, arg m4]

  trigger "showVehicleState" true [  arg (x      vehicleState)
                                   , arg (dx     vehicleState)
                                   , arg (y      vehicleState)
                                   , arg (dy     vehicleState)
                                   , arg (z      vehicleState)
                                   , arg (dz     vehicleState)
                                   , arg (phi    vehicleState)
                                   , arg (dphi   vehicleState)
                                   , arg (theta  vehicleState)
                                   , arg (dtheta vehicleState)
                                   , arg (psi    vehicleState)
                                   , arg (dpsi   vehicleState)
                                  ]

-- Compile the spec
main = reify spec >>= compile "hackflight"
