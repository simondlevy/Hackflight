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

counter :: Stream Bool -> Stream Bool -> Stream Int32
counter inc reset = cnt
  where
   cnt = if reset then 0 else if inc then z + 1 else z
   z = [0] ++ cnt

yawController :: Stream Double -> Stream Double -> Stream Double -> PidController

yawController kp ki windupMax = 
    makePidController (yawFun kp ki windupMax) (YawState 0)

yawFun :: Stream Double -> Stream Double -> Stream Double -> PidFun
yawFun kp ki windupMax vehicleState demands controllerState =

    -- Return updated demands and controller state
    (Demands 0 0 0 (kp * error' + ki * errorIntegral), YawState errorIntegral)

    -- Compute error as target minus actual
    where error' = (yaw demands) - (dpsi vehicleState)

          -- Accumualte error integral
          errorIntegral = constrain_abs (z + error') windupMax
          z = [0] ++ (yawErrorIntegral controllerState)

