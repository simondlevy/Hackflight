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

import VehicleState
import Sensor

time :: Stream Float
time  = extern "copilot_time" Nothing

altimeterZ :: Stream Float
altimeterZ = extern "copilot_altimeterZ" Nothing


altimeter :: Sensor

altimeter vehicleState =

  VehicleState (x      vehicleState)
               (dx     vehicleState)
               (y      vehicleState)
               (dy     vehicleState)
               ((z     vehicleState) + altimeterZ)
               ((dz    vehicleState) + ((altimeterZ - z') / (time - time')))
               (phi    vehicleState)
               (dphi   vehicleState)
               (theta  vehicleState)
               (dtheta vehicleState)
               (psi    vehicleState)
               (dpsi   vehicleState)

  where 
    
    z' = [0] ++ altimeterZ
    time' = [0] ++ time
