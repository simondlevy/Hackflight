{--
  Hackflight for simulators

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module HackflightSim where

import Language.Copilot

import Hackflight
import Receiver
import Sensor
import PidController
import Mixer
import Safety

hackflightSim :: Receiver -> [Sensor] -> [PidFun] -> Mixer -> Motors

hackflightSim receiver sensors pidfuns mixer = motors

   where

     (motors, _) = hackflight receiver sensors pidfuns mixer getSafetySim
