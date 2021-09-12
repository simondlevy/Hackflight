{--
  PID control for yaw angular velocity

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module YawPid

where

import Language.Copilot

import State(dpsi)
import PidController
import Demands
import Utils(constrain_abs)

yawController :: Stream Float -> Stream Float -> Stream Float -> PidController

yawController kp ki windupMax = makePidController (yawFun kp ki windupMax)


yawFun :: Stream Float -> Stream Float -> Stream Float -> PidFun

yawFun kp ki windupMax state demands =

    Demands 0 0 0 (kp * error' + ki * errorIntegral)

    where 

      error' = (yaw demands) - (dpsi state)

      -- Accumualte error integral
      errorIntegral = constrain_abs (errorIntegral' + error') windupMax

      -- Maintain controller state between calls
      errorIntegral' = [0] ++ errorIntegral
