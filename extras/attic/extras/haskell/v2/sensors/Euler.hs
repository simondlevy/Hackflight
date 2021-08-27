{--
  Haskell Copilot support for Euler angles, until we can get Quaternion working

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE DataKinds        #-}

module Euler

where

import Language.Copilot

import VehicleState
import Sensor

eulerX :: Stream Double
eulerX = extern "eulerX" Nothing

eulerY :: Stream Double
eulerY = extern "eulerY" Nothing

eulerZ :: Stream Double
eulerZ = extern "eulerZ" Nothing

euler :: Sensor

euler vehicleState = 

  VehicleState (x      vehicleState)
               (dx     vehicleState)
               (y      vehicleState)
               (dy     vehicleState)
               (z      vehicleState) 
               (dz     vehicleState) 
               ((phi   vehicleState) + eulerX)
               (dphi   vehicleState)
               ((theta vehicleState) + eulerY)
               (dtheta vehicleState)
               ((psi   vehicleState) + eulerZ)
               (dpsi   vehicleState)
