{--
  Haskell Copilot support for sensors

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

{-# LANGUAGE DataKinds        #-}

module Gyrometer

where

import Language.Copilot

import Sensors

gyroModifyState :: SensorFun

modifyState (Gyrometer gx gy gz) vehicleState = 

  VehicleState (x vehicleState)
               (dx vehicleState)
               (y vehicleState)
               (dy vehicleState)
               (z vehicleState)
               (dz vehicleState)
               (phi vehicleState)
               gx
               (theta vehicleState)
               gy
               (psi vehicleState)
               gz

gyrometerValues :: Stream (Array 3 Double)
gyrometerValues  = extern "gyrometerValues" Nothing
