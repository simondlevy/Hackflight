{--
  Hackflight core algorithm

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module Hackflight where

import Language.Copilot

import Prelude hiding((||), (++), (<), (&&), (==), div, mod)

import Receiver
import State
import Sensor
import PidController
import Demands
import Mixer
import Utils(compose, isTrue)
import Time(time_msec)

hackflight :: Receiver -> [Sensor] -> [PidController] -> Mixer ->
  (Motors, Stream Bool, Stream Bool)

hackflight receiver sensors pidControllers mixer = (motors, ledState, failsafe)

  where

    -- Use receiver data to trap failsafe
    failsafe = receiverLostSignal || failsafe' where failsafe' = [False] ++ failsafe

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

    -- Map throttle from [-1,+1] to [0,1]
    demands' = Demands (((throttle demands) + 1) / 2)
                       (roll demands)
                       (pitch demands)
                       (yaw demands)

    -- Apply mixer to demands to get motor values, returning motor values and LED state
    motors = mixer failsafe demands

    -- Blink LED on startup
    ledState = time_msec < 2000 && mod (div time_msec 50) 2 == 0
