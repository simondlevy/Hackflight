{--
  Hackflight core algorithm

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module Hackflight where

import Language.Copilot

import Prelude hiding((||), (++), (<), (>), (&&), (==), div, mod, not)

import Safety
import Receiver
import State
import Sensor
import PidController
import Demands
import Mixer
import Serial
import Utils(compose)

hackflight :: Receiver -> [Sensor] -> [PidController] -> Mixer -> SafetyFun
  -> (Motors, State, Safety)

hackflight receiver sensors pidControllers mixer safetyFun
   = (motors, vehicleState, safety)

  where

    -- Get receiver demands from external C functions
    receiverDemands = getDemands receiver

    -- Inject the receiver demands into the PID controllers
    pidControllers' = map (\p -> PidController (pidFun p) receiverDemands) pidControllers

    -- Get the vehicle state by running the sensors
    vehicleState = compose sensors zeroState

    -- Map the PID update function to the pid controllers
    pidControllers'' = map (pidUpdate vehicleState) pidControllers'

    -- Sum over the list of demands to get the final demands
    demands = foldr addDemands receiverDemands (map pidDemands pidControllers'')

    -- Get safety status
    safety = safetyFun vehicleState

    -- Apply mixer to demands to get motor values, returning motor values and LED state
    motors = mixer safety demands
