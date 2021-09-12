{--
  PID controller for position hold

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module PosHoldPid(posHoldController)

where

import Language.Copilot

import State(dx, dy, psi)
import PidController
import Demands
import Utils(in_band)

posHoldController :: Stream Float -> Stream Float -> PidController

posHoldController kp stickDeadband = makePidController (posHoldFun kp stickDeadband)


posHoldFun :: Stream Float -> Stream Float -> PidFun

posHoldFun kp stickDeadband state demands =

  -- Demands 0 rollDemand pitchDemand 0
  Demands 0 rollDemand pitchDemand 0

  where

    compute demand error' = if in_band demand stickDeadband then (-kp) * error' else 0

    -- Rotate X, Y velocities into body frame
    psi' = psi state
    cp = cos psi'
    sp = sin psi'
    dx' = dx state
    dy' = dy state

    rollDemand = compute (roll demands) (cp * dy' - sp * dx')

    pitchDemand = compute (pitch demands) (cp * dx' + sp * dy')
