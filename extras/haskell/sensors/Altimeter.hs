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
time  = extern "time" Nothing

altimeterZ :: Stream Double
altimeterZ = extern "altimeterZ" Nothing

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

  where

    altimeterDz = (altimeterZ - altimeterZ') / 0.0002 -- (time - time')

    altimeterZ' = [0] ++ altimeterZ
    time' = [0] ++ time
