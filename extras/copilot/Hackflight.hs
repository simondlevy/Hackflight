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

import PidController
--import YawPid
import AltHoldPid

spec = do

  -- Initialization --------------------------------------------

  let sensors = [euler, gyrometer, altimeter]

  let altHold = altHoldController 0.75 -- Kp
                                  1.5  -- Ki
                                  0.4  -- windupMax
                                  2.5  -- pilotVelZMax
                                  0.2  -- stickDeadband
  let pidControllers = [altHold]

  let mixer = QuadXAPMixer

  -- Main algorithm ---------------------------------------------

  -- Get receiver demands from external C functions
  let receiverDemands = Demands receiverThrottle receiverRoll receiverPitch receiverYaw

  -- Inject the receiver demands into the PID controllers
  let pidControllers' = map (\p -> PidController (pidFun p) receiverDemands)
                            pidControllers

  -- Get the vehicle state by running the sensors
  let vehicleState = compose sensors zeroVehicleState

  -- Map the PID update function to the pid controllers
  let pidControllers'' = map (pidUpdate vehicleState) pidControllers'

  -- Sum over the list of demands to get the final demands
  let demands = foldr addDemands zeroDemands (map pidDemands pidControllers'')

  -- Use the mixer to convert the demands into motor values
  let m1 = getMotor1 mixer demands
  let m2 = getMotor2 mixer demands
  let m3 = getMotor3 mixer demands
  let m4 = getMotor4 mixer demands

  -- Send the motor values to the external C function
  trigger "runMotors" true [arg m1, arg m2, arg m3, arg m4]

-- Compile the spec
main = reify spec >>= compile "hackflight"
