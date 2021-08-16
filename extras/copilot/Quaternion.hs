{--
  Haskell Copilot support for using quaternion as a sensor

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

{-# LANGUAGE DataKinds        #-}

module Gyrometer

where

import Language.Copilot

import VehicleState

quat :: Stream (Array 4 Double)
quat = extern "quatValues" Nothing

quatModifyState :: VehicleState

quatModifyState = VehicleState 0 0 0 0 0 0 phi 0 theta 0 psi 0

  where qw = quat.!!0
        qx = quat.!!1
        qy = quat.!!2
        qz = quat.!!3
        phi = 0
        theta = 0
        psi = 0

        -- ex = atan2(2.0f*(qw*qx+qy*qz), qw*qw-qx*qx-qy*qy+qz*qz);
        -- ey = asin(2.0f*(qx*qz-qw*qy));
        -- ez = atan2(2.0f*(qx*qy+qw*qz), qw*qw+qx*qx-qy*qy-qz*qz);
