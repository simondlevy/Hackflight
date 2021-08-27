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

flowX :: Stream Float
flowX = extern "copilot_flowX" Nothing

flowY :: Stream Float
flowY = extern "copilot_flowY" Nothing

opticalFlow :: Sensor

opticalFlow vehicleState = 

  VehicleState (x      vehicleState)
               ((dx    vehicleState) + dx')
               (y      vehicleState)
               ((dy    vehicleState) + dy')
               (z      vehicleState) 
               (dz     vehicleState) 
               (phi    vehicleState)
               (dphi   vehicleState)
               (theta  vehicleState)
               (dtheta vehicleState)
               (psi    vehicleState) 
               (dpsi   vehicleState)

  where

    -- Rotate flow velocity from body frame to earth frame
    
    psi' = psi vehicleState

    cp = cos psi'
    sp = sin psi'

    -- dx' = flowX * cp - flowY * sp;
    dx' = psi'
    dy' = flowX * sp + flowY * cp;
