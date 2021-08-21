{--
  PID controller for position hold

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module PosHoldPid(posHoldController)

where

import Language.Copilot

import VehicleState(dx, dy)
import PidController
import Demands
import Utils(in_band)

posHoldController :: Stream Double -> Stream Double -> PidController

posHoldController kp stickDeadband = makePidController (posHoldFun kp stickDeadband)

posHoldFun :: Stream Double -> Stream Double -> PidFun
posHoldFun kp stickDeadband vehicleState demands =

    Demands 0 roll' pitch' 0

    where

      (roll', pitch') = (0, 0)

{--
      if in_band (roll demands) stickDeadband &&
                           in_band (pitch demands) stickDeadband
    
                        then

                            let p = psi vehicleState

                                cp = cos p
                                sp = sin p

                                dx' = dx vehicleState
                                dy' = dy vehicleState

                              -- Rotate X, Y velocities into body frame
                            in (-kp * (cp * dy' - sp * dx'),
                                -kp * (cp * dx' + sp * dy'))

                        else (0, 0)
--}
