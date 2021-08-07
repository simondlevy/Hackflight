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

yawController :: Double -> Double -> Double -> PidController

yawController kp ki windupMax = 
    PidController (yawFun kp ki windupMax) (YawState 0)

yawFun :: Double -> Double -> Double -> PidFun
yawFun kp ki windupMax vehicleState demands controllerState =

    -- Compute error as target minus actual
    let err = (yaw demands) - (dpsi vehicleState)

        -- Accumualte error integral
        errI = constrain_abs ((yawErrorIntegral controllerState) + err) windupMax

    -- Return updated demands and controller state
    in (Demands (throttle demands) 
                (roll demands)
                (pitch demands)
                (kp * err + ki * errI),
        YawState errI)
