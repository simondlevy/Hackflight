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

import Receiver
import Demands
import VehicleState
import Utils(compose)

import Altimeter
import Gyrometer
import Euler

import AltHoldPid

spec = do

  let sensors = [euler, gyrometer, altimeter]

  -- Get the vehicle state by running the sensors
  let vehicleState = compose sensors initialVehicleState

  let altHold = altHoldController 0.75 -- Kp
                                  1.5  -- Ki
                                  0.4  -- windupMax
                                  2.5  -- pilotVelZMax
                                  0.2  -- stickDeadband

  let pidControllers = [altHold]

  let receiverDemands = Demands receiverThrottle receiverRoll receiverPitch receiverYaw

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
