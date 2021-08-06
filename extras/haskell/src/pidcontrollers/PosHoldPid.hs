{--
  PID controller for position hold

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module PosHoldPid(posHoldController)

where

import VehicleState
import PidControl
import Demands
import Utils(in_band)

posHoldController :: Double -> Double -> PidController

posHoldController kp stickDeadband =
    PidController (posHoldClosure kp stickDeadband) NoState

posHoldClosure :: Double -> Double -> PidFun
posHoldClosure kp stickDeadband vehicleState demands _controllerState =

    let newDemands = if in_band (Demands.roll demands) stickDeadband &&
                        in_band (Demands.pitch demands) stickDeadband
        
                    then

                        let p = VehicleState.psi vehicleState

                            -- Rotate X, Y velocities into body frame

                            cp = cos p
                            sp = sin p

                            xx = VehicleState.dx vehicleState
                            yy = VehicleState.dy vehicleState

                        in Demands (Demands.throttle demands)
                                   (-kp * (cp * yy - sp * xx))
                                   (-kp * (cp * xx + sp * yy))
                                   (Demands.yaw demands)

                    else demands

    in (newDemands, NoState)
