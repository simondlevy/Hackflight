{--
  Haskell Copilot support for gyrometers

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE DataKinds        #-}

module Gyrometer

where

import Language.Copilot

import State
import Sensor

gyrometerX :: Stream Float
gyrometerX = extern "copilot_gyrometerX" Nothing

gyrometerY :: Stream Float
gyrometerY = extern "copilot_gyrometerY" Nothing

gyrometerZ :: Stream Float
gyrometerZ = extern "copilot_gyrometerZ" Nothing

gyrometer :: Sensor

gyrometer state = 

  State (x       state)
               (dx      state)
               (y       state)
               (dy      state)
               (z       state) 
               (dz      state) 
               (phi     state)
               ((dphi   state) + gyrometerX)
               (theta   state)
               ((dtheta state) + gyrometerY)
               (psi     state)
               ((dpsi   state) + gyrometerZ)
