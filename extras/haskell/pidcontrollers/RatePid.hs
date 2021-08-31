{--
  PID control for roll/pitch angular velocity

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module RatePid(rateController)

where

import Language.Copilot

import State(dphi, dtheta)
import PidController
import Demands
import Utils(constrain_abs)

rateController :: Stream Float  -- kp
               -> Stream Float  -- ki
               -> Stream Float  -- kd
               -> Stream Float  -- windup max
               -> Stream Float  -- rate max
               -> PidController

rateController kp ki kd windupMax rateMax = 
    makePidController (rateFun kp ki kd windupMax rateMax)


rateFun :: Stream Float
        -> Stream Float
        -> Stream Float
        -> Stream Float
        -> Stream Float
        -> PidFun

rateFun kp ki kd windupMax rateMax state demands =

    let

        computeDemand demand value errorIntegral errorDelta1 errorDelta2 error' =
                      
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

              newErrorDelta = if reset then 0 else newError - error'

              newErrorDelta1 = if reset then 0 else errorDelta1

              -- Run a simple low-pass filter on the error derivative
              errorDerivative = if reset then 0 else
                                newErrorDelta + newErrorDelta1 + errorDelta2

        (rollDemand, rollError, rollErrorIntegral, rollErrorDelta, rollErrorDelta1) =
            computeDemand (roll demands)
                          (dphi state)
                          rollErrorIntegral'
                          rollErrorDelta1'
                          rollErrorDelta2'
                          rollError'

         -- Pitch demand is nose-down positive, so we negate pitch-forward
         -- (nose-down negative) to reconcile them
        (pitchDemand, pitchError, pitchErrorIntegral, pitchErrorDelta, pitchErrorDelta1) =
            computeDemand (pitch demands)
                          (-(dtheta state))
                          pitchErrorIntegral'
                          pitchErrorDelta1'
                          pitchErrorDelta2'
                          pitchError'

        -- Maintain controller state between calls

        rollError' = [0] ++ rollError
        rollErrorIntegral' = [0] ++ rollErrorIntegral
        rollErrorDelta1' = [0] ++ rollErrorDelta
        rollErrorDelta2' = [0] ++ rollErrorDelta1

        pitchError' = [0] ++ pitchError
        pitchErrorIntegral' = [0] ++ pitchErrorIntegral
        pitchErrorDelta1' = [0] ++ pitchErrorDelta
        pitchErrorDelta2' = [0] ++ pitchErrorDelta1
        
    in Demands 0 rollDemand pitchDemand 0

