{--
  Classic PID controller

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module FullPidController

where

import Language.Copilot
import Copilot.Compile.C99

import Utils(constrain_abs)

data FullPidState = 

    FullPidState { errorIntegral :: Stream Double,
                   deltaError1 :: Stream Double,
                   deltaError2 :: Stream Double,
                   errorPrev :: Stream Double }

computeDemand :: Stream Double ->  -- kp
                 Stream Double ->  -- ki
                 Stream Double ->  -- kd
                 Stream Double ->  -- windupMax
                 Stream Double ->  -- valueMax
                 Stream Double ->  -- demand
                 Stream Double ->  -- value
                 Stream Double     -- new value

computeDemand kp ki kd windupMax valueMax demand value =

    kp * error' + ki * errorIntegral + kd * errorDerivative

    where 

      error' = demand - value

      --  Reset PID state on quick value change
      reset = abs value > valueMax

      errorIntegral = if reset then 0 else
             constrain_abs (errorIntegralState + error') windupMax

      deltaError = if reset then 0 else error' - errorPrevState

      deltaError1 = if reset then 0 else deltaError1State

      -- Run a simple low-pass filter on the error derivative
      errorDerivative = if reset then 0 else
                        deltaError1 + deltaError2State + deltaError

      -- Maintain controller state between calls
      errorIntegralState = [0] ++ errorIntegral
      errorPrevState = [0] ++ error'
      deltaError1State = [0] ++ deltaError
      deltaError2State = [0] ++ deltaError1

