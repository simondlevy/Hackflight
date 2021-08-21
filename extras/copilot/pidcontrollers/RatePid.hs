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
import Demands
import Utils(constrain_abs)

rateController :: Stream Double  -- kp
               -> Stream Double  -- ki
               -> Stream Double  -- kd
               -> Stream Double  -- windup max
               -> Stream Double  -- rate max
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

    let

        computeDemand demand value errorIntegral errorDelta1 errorDelta2 errorPrev =
                      
            (kp * newError + ki * newErrorIntegral + kd * errorDerivative, 
             newError,
             newErrorIntegral,
             newErrorDelta,
             newErrorDelta1)

            where 

              newError = demand - value

              --  Reset PID state on quick value change
              reset = abs value > rateMax

              newErrorIntegral = if reset then 0 else
                                 constrain_abs (errorIntegral + newError) windupMax

              newErrorDelta = if reset then 0 else newError - errorPrev

              newErrorDelta1 = if reset then 0 else errorDelta1

              -- Run a simple low-pass filter on the error derivative
              errorDerivative = if reset then 0 else
                                newErrorDelta + newErrorDelta1 + errorDelta2

        (rollDemand, rollError, rollErrorIntegral, rollErrorDelta, rollErrorDelta1) =
            computeDemand (roll demands) (dphi vehicleState) rollErrorIntegralState rollErrorDelta1State rollErrorDelta2State rollErrorPrev

         -- Pitch demand is nose-down positive, so we negate pitch-forward
         -- (nose-down negative) to reconcile them
        (pitchDemand, pitchError, pitchErrorIntegral, pitchErrorDelta, pitchErrorDelta1) =
            computeDemand (pitch demands) (-(dtheta vehicleState)) pitchErrorIntegralState pitchErrorDelta1State pitchErrorDelta2State pitchErrorPrev

        -- Maintain controller state between calls

        rollErrorPrev = [0] ++ rollError
        rollErrorIntegralState = [0] ++ rollErrorIntegral
        rollErrorDelta1State = [0] ++ rollErrorDelta
        rollErrorDelta2State = [0] ++ rollErrorDelta1

        pitchErrorPrev = [0] ++ pitchError
        pitchErrorIntegralState = [0] ++ pitchErrorIntegral
        pitchErrorDelta1State = [0] ++ pitchErrorDelta
        pitchErrorDelta2State = [0] ++ pitchErrorDelta1
        
    in Demands 0 rollDemand pitchDemand 0

