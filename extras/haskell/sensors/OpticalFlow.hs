{--
  Haskell Copilot support for optical-flow sensors

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE DataKinds        #-}

module OpticalFlow

where

import Language.Copilot

import State
import Sensor

flowX :: Stream Float
flowX = extern "copilot_flowX" Nothing

flowY :: Stream Float
flowY = extern "copilot_flowY" Nothing

opticalFlow :: Sensor

opticalFlow state = 

  State (x      state)
               ((dx    state) + dx')
               (y      state)
               ((dy    state) + dy')
               (z      state) 
               (dz     state) 
               (phi    state)
               (dphi   state)
               (theta  state)
               (dtheta state)
               (psi    state) 
               (dpsi   state)

  where

    -- Rotate flow velocity from body frame to earth frame
    
    psi' = psi state

    cp = cos psi'
    sp = sin psi'

    dx' = flowX * cp - flowY * sp;
    dy' = flowX * sp + flowY * cp;
