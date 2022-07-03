{--
  Haskell Copilot support for using quaternion as a sensor

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module Quaternion

where

import Language.Copilot hiding(atan2)
import Copilot.Language.Stream
import Prelude hiding(atan2)

import State
import Sensor
import Utils

quaternion :: Sensor

quaternion state  =

  State (x      state)
        (dx     state)
        (y      state)
        (dy     state)
        (z      state) 
        (dz     state) 
        (update phi' phi)
        (dphi   state) 
        (update theta' theta)
        (dtheta state)
        (update psi' psi)
        (dpsi   state)

  where 

    update newval old = if qavail then newval else (old state)

    phi' = atan2 (2*(qw*qx+qy*qz)) (qw*qw-qx*qx-qy*qy+qz*qz)
    theta' = -(asin (2*(qx*qz-qw*qy)))
    psi' = atan2 (2*(qx*qy+qw*qz))  (qw*qw+qx*qx-qy*qy-qz*qz)

----------------------------------------------------------------------

qavail :: SBool
qavail = extern "imuGotQuaternion" Nothing

qw :: SFloat
qw = extern "imuQuaternionW" Nothing

qx :: SFloat
qx = extern "imuQuaternionX" Nothing

qy :: SFloat
qy = extern "imuQuaternionY" Nothing

qz :: SFloat
qz = extern "imuQuaternionZ" Nothing
