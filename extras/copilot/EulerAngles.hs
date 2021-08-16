{--
  Fake Euler-angle "sensor" until we can get eulerernion working

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

{-# LANGUAGE DataKinds        #-}

module EulerAngles

where

import Language.Copilot

import VehicleState

euler :: Stream (Array 3 Double)
euler = extern "eulerValues" Nothing

eulerModifyState :: VehicleState

eulerModifyState = VehicleState 0 0 0 0 0 0 phi 0 theta 0 psi 0

  where phi = euler.!!0
        theta = euler.!!1
        psi = euler.!!2
