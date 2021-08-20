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
import Mixer
import Utils

import Altimeter
import Gyrometer
import Euler

--import PidController
--import YawPid
--import AltHoldPid

spec = do

  -- Initialization --------------------------------------------

  let sensors = [euler, gyrometer, altimeter]

  let mixer = QuadXAPMixer

  let kp = 0.75
  let ki =  1.5 
  let windupMax = 0.4
  let pilotVelZMax =  2.5
  let stickDeadband = 0.2

  -- Main algorithm ---------------------------------------------

  -- Get the receiver demands
  let receiverDemands = Demands receiverThrottle receiverRoll receiverPitch receiverYaw

  -- Get the vehicle state by running the sensors
  let vehicleState = compose sensors zeroVehicleState
  
  let demands = receiverDemands

  -- NED => ENU
  let altitude = -(z vehicleState)

  let throttleDemand = throttle demands

  let targetVelocity = pilotVelZMax * throttleDemand

  -- Compute error as altTarget velocity minus actual velocity, after
  -- negating actual to accommodate NED
  let error' = targetVelocity + (dz vehicleState)

  trigger "debug" true [arg error'] 

  let m1 = getMotor1 mixer demands
  let m2 = getMotor2 mixer demands
  let m3 = getMotor3 mixer demands
  let m4 = getMotor4 mixer demands

  trigger "runMotors" true [arg m1, arg m2, arg m3, arg m4]

-- Compile the spec
main = reify spec >>= compile "hackflight"
