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

time :: Stream Double
time  = extern "copilot_time" Nothing

altimeterZ :: Stream Double
altimeterZ = extern "copilot_altimeterZ" Nothing
altimeterDz :: Stream Double
altimeterDz = extern "copilot_altimeterDz" Nothing


altimeter :: Sensor

altimeter vehicleState =

  VehicleState (x      vehicleState)
               (dx     vehicleState)
               (y      vehicleState)
               (dy     vehicleState)
               ((z     vehicleState) + altimeterZ)
               ((dz    vehicleState) + altimeterDz)
               (phi    vehicleState)
               (dphi   vehicleState)
               (theta  vehicleState)
               (dtheta vehicleState)
               (psi    vehicleState)
               (dpsi   vehicleState)
