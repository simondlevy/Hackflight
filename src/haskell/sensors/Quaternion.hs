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

quaternion :: Sensor

quaternion state  =

  State (x      state)
        (dx     state)
        (y      state)
        (dy     state)
        (z      state) 
        (dz     state) 
        ((phi   state) + phi')
        (dphi   state) 
        ((theta state) + theta')
        (dtheta state)
        ((psi   state) + psi')
        (dpsi   state)

  where 

    qw = quaternionW
    qx = quaternionX
    qy = quaternionY
    qz = quaternionZ
    phi' = atan2 (2*(qw*qx+qy*qz)) (qw*qw-qx*qx-qy*qy+qz*qz)
    theta' = -(asin (2*(qx*qz-qw*qy)))
    psi' = atan2 (2*(qx*qy+qw*qz))  (qw*qw+qx*qx-qy*qy-qz*qz)

----------------------------------------------------------------------

quaternionW :: Stream Float
quaternionW = extern "stream_imuQuaternionW" Nothing

quaternionX :: Stream Float
quaternionX = extern "stream_imuQuaternionX" Nothing

quaternionY :: Stream Float
quaternionY = extern "stream_imuQuaternionY" Nothing

quaternionZ :: Stream Float
quaternionZ = extern "stream_imuQuaternionZ" Nothing
