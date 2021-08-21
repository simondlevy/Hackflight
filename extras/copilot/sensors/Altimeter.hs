{--
  Haskell Copilot support for altimeters

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

{-# LANGUAGE DataKinds        #-}

module Altimeter

where

import Language.Copilot

import VehicleState
import Sensor

altimeterZ :: Stream Double
altimeterZ = extern "altimeterZ" Nothing

altimeterDz :: Stream Double
altimeterDz = extern "altimeterDz" Nothing

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
