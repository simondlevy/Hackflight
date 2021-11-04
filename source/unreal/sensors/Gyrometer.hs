{--
  Haskell Copilot support for gyrometers

  Copyright(C) 2021 on D.Levy

  MIT License
--}

module Gyrometer

where

import Language.Copilot

import State
import Sensor

gyrometer :: Sensor

gyrometer state = 

  State (x       state)
        (dx      state)
        (y       state)
        (dy      state)
        (z       state) 
        (dz      state) 
        (phi     state)
        ((dphi   state) + (deg2rad gyrometerX))
        (theta   state)
        ((dtheta state) + (deg2rad gyrometerY))
        (psi     state)
        ((dpsi   state) + (deg2rad gyrometerZ))

  where deg2rad d = d * pi / 180

----------------------------------------------------------------------

gyrometerX :: Stream Float
gyrometerX = extern "stream_gyrometerX" Nothing

gyrometerY :: Stream Float
gyrometerY = extern "stream_gyrometerY" Nothing

gyrometerZ :: Stream Float
gyrometerZ = extern "stream_gyrometerZ" Nothing
