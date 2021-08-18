{--
  Classic PID controller

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module FullPidController

where

import Utils(constrain_abs)

data FullPidState = 

    FullPidState { errorIntegral :: Double,
                   deltaError1 :: Double,
                   deltaError2 :: Double,
                   errorPrev :: Double }

initFullPidState :: FullPidState

initFullPidState = FullPidState 0 0 0 0

computeDemand :: Double ->
                 Double ->
                 Double ->
                 Double ->
                 Double ->
                 FullPidState -> 
                 Double ->
                 Double ->
                 (Double, FullPidState)

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

