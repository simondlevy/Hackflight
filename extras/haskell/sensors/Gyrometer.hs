{--
  Haskell Copilot support for gyrometers

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE DataKinds        #-}

module Gyrometer

where

import Language.Copilot

import VehicleState
import Sensor

gyrometerX :: Stream Double
gyrometerX = extern "gyrometerX" Nothing

gyrometerY :: Stream Double
gyrometerY = extern "gyrometerY" Nothing

gyrometerZ :: Stream Double
gyrometerZ = extern "gyrometerZ" Nothing

gyrometer :: Sensor

gyrometer vehicleState = 

  VehicleState (x       vehicleState)
               (dx      vehicleState)
               (y       vehicleState)
               (dy      vehicleState)
               (z       vehicleState) 
               (dz      vehicleState) 
               (phi     vehicleState)
               ((dphi   vehicleState) + gyrometerX)
               (theta   vehicleState)
               ((dtheta vehicleState) + gyrometerY)
               (psi     vehicleState)
               ((dpsi   vehicleState) + gyrometerZ)
