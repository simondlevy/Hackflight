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

yawController kp ki (state, demands, ready) = demands'

  where 

    -- Compute error as difference between yaw demand and angular velocity
    err = (yaw demands) - (dpsi state)

    -- Accumulate I term, resetting on large yaw jump
    errI = if abs err > (deg2rad dpsMax) then 0
            else if ready then constrain_abs (errI + err)  windupMax
            else [0] ++ errI

    demands' = Demands (throttle demands)
                       (roll demands)
                       (pitch demands)
                       (kp * err + ki * errI)

