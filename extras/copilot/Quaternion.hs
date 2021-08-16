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
gyro = extern "gyro" Nothing

gyroModifyState :: VehicleState
gyroModifyState = VehicleState 0 0 0 0 0 0 0 (gyro.!!0) 0 (gyro.!!1) 0 (gyro.!!2)
