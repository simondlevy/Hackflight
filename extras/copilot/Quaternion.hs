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
quatModifyState = VehicleState 0 0 0 0 0 0 0 0 0 0 0 0
