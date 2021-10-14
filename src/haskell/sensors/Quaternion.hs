{--
  Haskell Copilot support for using quaternion as a sensor

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module Quaternion

where

import Language.Copilot

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
    phi' = -(atan2' (2*(qw*qx+qy*qz)) (qw*qw-qx*qx-qy*qy+qz*qz))
    theta' = asin (2*(qx*qz-qw*qy))
    psi' = atan2' (2*(qx*qy+qw*qz))  (qw*qw+qx*qx-qy*qy-qz*qz)

----------------------------------------------------------------------

-- XXX eventually should be part of Copilot
-- https://en.wikipedia.org/wiki/Atan2#Definition_and_computation
atan2' :: Stream Float -> Stream Float -> Stream Float
atan2' y x = 
  if x > 0 then atan (y/x)
  else if x < 0 && y >= 0 then atan(y/x) + pi
  else if x < 0 && y < 0 then atan(y/x) - pi
  else if x == 0 && y > 0 then pi
  else if x == 0 && y < 0 then (-pi) 
  else 0 -- undefined

----------------------------------------------------------------------

quaternionW :: Stream Float
quaternionW = extern "copilot_quaternionW" Nothing

quaternionX :: Stream Float
quaternionX = extern "copilot_quaternionX" Nothing

quaternionY :: Stream Float
quaternionY = extern "copilot_quaternionY" Nothing

quaternionZ :: Stream Float
quaternionZ = extern "copilot_quaternionZ" Nothing
