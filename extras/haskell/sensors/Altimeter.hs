{--
  Haskell Copilot support for altimeters

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}
{-# LANGUAGE DataKinds        #-}

module Altimeter

where

import Language.Copilot

import State
import Sensor
import Time(time_sec)

altimeterZ :: Stream Float
altimeterZ = extern "copilot_altimeterZ" Nothing

altimeter :: Sensor

altimeter state =

  State (x      state)
               (dx     state)
               (y      state)
               (dy     state)
               ((z     state) + altimeterZ)
               ((dz    state) + ((altimeterZ - z') / (time_sec - time_sec')))
               (phi    state)
               (dphi   state)
               (theta  state)
               (dtheta state)
               (psi    state)
               (dpsi   state)

  where 
    
    z' = [0] ++ altimeterZ
    time_sec' = [0] ++ time_sec
