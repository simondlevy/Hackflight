{--
  PID control for roll/pitch angular velocity

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module RatePid(rateController)

where

import Language.Copilot

import VehicleState
import PidController
import FullPidController
import Demands

rateController :: Stream Double 
               -> Stream Double
               -> Stream Double
               -> Stream Double
               -> Stream Double
               -> PidController

rateController kp ki kd windupMax rateMax = 
    makePidController (rateFun kp ki kd windupMax rateMax)


rateFun :: Stream Double
        -> Stream Double
        -> Stream Double
        -> Stream Double
        -> Stream Double
        -> PidFun

rateFun kp ki kd windupMax rateMax vehicleState demands =

    Demands 0 rollDemand pitchDemand 0

    where

      rollDemand = 0
      pitchDemand = 0
