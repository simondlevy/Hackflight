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

quat :: Stream (Array 4 Double)
quat = extern "quatValues" Nothing

quatModifyState :: VehicleState

quatModifyState = VehicleState 0 0 0 0 0 0 0 0 theta 0 psi 0

  where qw = quat.!!0
        qx = quat.!!1
        qy = quat.!!2
        qz = quat.!!3
        phi = atan2 (2*(qw*qx+qy*qz)) (qw*qw-qx*qx-qy*qy+qz*qz)
        theta = asin (2*(qx*qz-qw*qy))
        psi = atan2 (2*(qx*qy+qw*qz)  qw*qw+qx*qx-qy*qy-qz*qz)
