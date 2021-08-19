{--
  PID control for yaw angular velocity

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}
{-# LANGUAGE DataKinds        #-}

module YawPid

where

import Language.Copilot
import Copilot.Compile.C99

import VehicleState
import PidControllers
import Demands
import Utils(constrain_abs)

yawController :: Stream Double -> Stream Double -> Stream Double -> PidController

yawController kp ki windupMax = 
    makePidController (yawFun kp ki windupMax) (YawState 0)

yawFun :: Stream Double -> Stream Double -> Stream Double -> PidFun
yawFun kp ki windupMax vehicleState demands controllerState =

    -- Compute error as target minus actual
    let err = (yaw demands) - (dpsi vehicleState)

        -- Accumualte error integral
        errI = constrain_abs ((yawErrorIntegral controllerState) + err) windupMax

    -- Return updated demands and controller state
    in (Demands 0 0 0 (kp * err + ki * errI), YawState errI)
