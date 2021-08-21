{--
  PID controller for position hold

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module PosHoldPid(posHoldController)

where

import Language.Copilot

import VehicleState(dx, dy, psi)
import PidController
import Demands
import Utils(in_band)

posHoldController :: Stream Double -> Stream Double -> PidController

posHoldController kp stickDeadband = makePidController (posHoldFun kp stickDeadband)


posHoldFun :: Stream Double -> Stream Double -> PidFun

posHoldFun kp stickDeadband vehicleState demands =

  -- Demands 0 rollDemand pitchDemand 0
  Demands 0 rollDemand pitchDemand 0

  where

    compute demand error' = 
      
      if in_band demand stickDeadband then (-kp) * error' else 0

    -- Rotate X, Y velocities into body frame
    p = psi vehicleState
    cp = cos p
    sp = sin p
    dx' = dx vehicleState
    dy' = dy vehicleState

    rollDemand = compute (roll demands) (cp * dy' - sp * dx')

    pitchDemand = compute (pitch demands) (cp * dx' + sp * dy')
