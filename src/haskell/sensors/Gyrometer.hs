{--
  Haskell Copilot support for gyrometers

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module Gyrometer

where

import Language.Copilot

import State
import Sensor
import Utils

gyrometer :: Sensor

gyrometer state = 

  State (x       state)
        (dx      state)
        (y       state)
        (dy      state)
        (z       state) 
        (dz      state) 
        (phi     state)
        (update gx dphi)
        (theta   state)
        (update gy dtheta)
        (psi     state)
        (update gz dpsi)

  where 

    update newval old = if gavail then (deg2rad newval) else (old state)

    deg2rad d = d * pi / 180

----------------------------------------------------------------------

gavail :: SBool
gavail = extern "stream_imuGotGyrometer" Nothing

gx :: SFloat
gx = extern "stream_imuGyrometerX" Nothing

gy :: SFloat
gy = extern "stream_imuGyrometerY" Nothing

gz :: SFloat
gz = extern "stream_imuGyrometerZ" Nothing
