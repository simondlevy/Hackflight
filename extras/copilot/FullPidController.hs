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

initFullPidState :: FullPidState

initFullPidState = FullPidState 0 0 0 0

computeDemand :: Stream Double ->  -- kp
                 Stream Double ->  -- ki
                 Stream Double ->  -- kd
                 Stream Double ->  -- windupMax
                 Stream Double ->  -- valueMax
                 FullPidState ->   
                 Stream Double ->  -- demand
                 Stream Double ->  -- value
                 (Stream Double, FullPidState)

computeDemand kp ki kd windupMax valueMax pidState demand value =

    let 

        error' = demand - value

        --  Reset PID state on quick value change
        reset = abs value > valueMax

        errorIntegral' = if reset then 0 else
               constrain_abs ((errorIntegral pidState) + error') windupMax

        deltaError = if reset then 0 else error' - (errorPrev pidState)

        deltaError1' = if reset then 0 else deltaError1 pidState

        -- Run a simple low-pass filter on the error derivative
        errorDerivative = if reset then 0 else
                          deltaError1' + (deltaError2 pidState) + deltaError

    in (kp * error' + ki * errorIntegral' + kd * errorDerivative, 
       FullPidState errorIntegral' deltaError  deltaError1' error')
