{--
  PID control for yaw angular velocity

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module YawPid(yawController)

where

import Language.Copilot

import State(dpsi)
import PidController
import Demands
import Utils

windupMax = 6.0 :: SFloat
dpsMax = 40 :: SFloat -- deg/sec

yawController :: SFloat -> SFloat -> PidFun

yawController kp ki (state, demands) = (state, demands')

  where 

    demands' = Demands (throttle demands) (roll demands) (pitch demands) yawDemand

    yawDemand = kp * yawError + ki * yawErrorI

    yawError = (yaw demands) - (dpsi state)

    -- Accumualte error integral
    yawErrorI = constrain_abs (yawErrorI' + yawError) windupMax

    -- Maintain controller state between calls, resetting on quick change
    yawErrorI' = if (abs yawError) > (deg2rad dpsMax) then 0 else [0] ++ yawErrorI
