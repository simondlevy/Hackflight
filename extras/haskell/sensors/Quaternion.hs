{--
  Haskell Copilot support for using quaternion as a sensor

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

{-# LANGUAGE DataKinds        #-}

module Quaternion

where

import Language.Copilot

import VehicleState
import Sensor

quaternionW :: Stream Double
quaternionW = extern "quaternionW" Nothing

quaternionX :: Stream Double
quaternionX = extern "quaternionX" Nothing

quaternionY :: Stream Double
quaternionY = extern "quaternionY" Nothing

quaternionZ :: Stream Double
quaternionZ = extern "quaternionZ" Nothing

quaternion :: Sensor

quatModifyState vehicleState  =

  where qw = quaternionW
        qx = quaternionX
        qy = quaternionY
        qz = quaternionZ
        phi = atan2 (2*(qw*qx+qy*qz)) (qw*qw-qx*qx-qy*qy+qz*qz)
        theta = asin (2*(qx*qz-qw*qy))
        psi = atan2 (2*(qx*qy+qw*qz)  qw*qw+qx*qx-qy*qy-qz*qz)
