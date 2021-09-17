{--
  Hackflight for simulators

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module HackflightSim where

import Language.Copilot

import Prelude hiding((||), (++), (<), (>), (&&), (==), div, mod, not)

import Hackflight
import Safety
import Receiver
import State
import Sensor
import PidController
import Demands
import Mixer
import Time(time_msec)
import Serial
import Utils(compose)

hackflightSim :: Receiver -> [Sensor] -> [PidController] -> Mixer -> Motors

hackflightSim receiver sensors pidControllers mixer = motors
   where
     (motors, _, _) = hackflight receiver sensors pidControllers mixer getSafetySim
