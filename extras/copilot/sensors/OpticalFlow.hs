{--
  Haskell Copilot support for optical-flow sensors

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE DataKinds        #-}

module OpticalFlow

where

import Language.Copilot

import VehicleState
import Sensor

flowX :: Stream Double
flowX = extern "flowX" Nothing

flowY :: Stream Double
flowY = extern "flowY" Nothing

opticalFlow :: Sensor

opticalFlow vehicleState = 

  VehicleState (x      vehicleState)
               (dx     vehicleState)
               (y      vehicleState)
               (dy     vehicleState)
               (z      vehicleState) 
               (dz     vehicleState) 
               (phi    vehicleState)
               (dphi   vehicleState)
               (theta  vehicleState)
               (dtheta vehicleState)
               (psi    vehicleState) 
               (dpsi   vehicleState)
