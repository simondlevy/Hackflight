{--
  PID controller for position hold

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module PosHoldPid(posHoldController)

where

import VehicleState
import PidControllers
import Demands
import Utils(in_band)

posHoldController :: Double -> Double -> PidController

posHoldController kp stickDeadband =
    makePidController (posHoldFun kp stickDeadband) NoState

posHoldFun :: Double -> Double -> PidFun
posHoldFun kp stickDeadband vehicleState demands _controllerState =

    let (roll', pitch') = if in_band (roll demands) stickDeadband &&
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

    in (Demands 0 roll' pitch' 0, NoState)
