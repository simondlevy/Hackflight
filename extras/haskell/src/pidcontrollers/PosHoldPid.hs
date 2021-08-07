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
    PidController (posHoldFun kp stickDeadband) NoState

posHoldFun :: Double -> Double -> PidFun
posHoldFun kp stickDeadband vehicleState demands _controllerState =

    let newDemands = if in_band (roll demands) stickDeadband &&
                        in_band (pitch demands) stickDeadband
        
                    then

                        let p = psi vehicleState

                            -- Rotate X, Y velocities into body frame

                            cp = cos p
                            sp = sin p

                            xx = dx vehicleState
                            yy = dy vehicleState

                        in Demands (throttle demands)
                                   (-kp * (cp * yy - sp * xx))
                                   (-kp * (cp * xx + sp * yy))
                                   (yaw demands)

                    else demands

    in (newDemands, NoState)
