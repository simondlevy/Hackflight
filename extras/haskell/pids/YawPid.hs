{--
  PID control for yaw angular velocity

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module YawPid(yawController)

where

import VehicleState
import PidControl
import Demands
import Utils(constrain_abs)

import Debugging

yawController :: Double -> Double -> Double -> PidController

yawController kp ki windupMax = 
    PidController (yawClosure kp ki windupMax) (YawState 0)

yawClosure :: Double -> Double -> Double -> PidFun
yawClosure kp ki windupMax =

    \vehicleState -> \demands -> \controllerState ->

    -- Compute error as target minus actual
    let err = (Demands.yaw demands) - (VehicleState.dpsi vehicleState)

        -- Accumualte error integral
        errI = constrain_abs ((yawErrorIntegral controllerState) + err)
                             windupMax

    -- Return updated demands and controller state
    in (Demands (Demands.throttle demands) 
                (debug "->yaw: " (Demands.roll demands))
                (Demands.pitch demands)
                (kp * err + ki * errI),
        YawState errI)
