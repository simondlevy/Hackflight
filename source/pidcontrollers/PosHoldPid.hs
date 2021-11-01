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
import Utils

stickDeadband = 0.2 :: SFloat

posHoldController :: Stream Float -> PidFun

posHoldController kp (state, demands) = (state, demands')

  where

    demands' = Demands (throttle demands) rollDemand pitchDemand (yaw demands)

    rollDemand = compute roll (cp * dy' - sp * dx')

    pitchDemand = compute pitch (cp * dx' + sp * dy')

    compute dmdfun err = (dmdfun demands) + (if in_band (dmdfun demands) stickDeadband then (-kp) * err else 0)

    -- Rotate X, Y velocities into body frame
    psi' = psi state
    cp = cos psi'
    sp = sin psi'
    dx' = dx state
    dy' = dy state
