{--
  Haskell Copilot support for gyrometer

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

{-# LANGUAGE DataKinds        #-}

module Gyrometer

where

import Language.Copilot

import VehicleState

gyro :: Stream (Array 3 Double)
gyro = extern "gyroValues" Nothing

gyroModifyState :: VehicleState

gyroModifyState = VehicleState 0 0 0 0 0 0 0 dphi 0 dtheta 0 dpsi
  where dphi = gyro.!!0
        dtheta = gyro.!!1
        dpsi = gyro.!!2
