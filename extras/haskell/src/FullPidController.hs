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

        --  Reset PID state on quick value change
        pidState' = if abs(value) > valueMax
                     then (FullPidState 0 0 0 0)
                     else pidState

        err = demand - value

        errI = constrain_abs ((errorIntegral pidState') + err) windupMax

        deltaErr = err - (errorPrev pidState')

        -- Run a simple low-pass filter on the error derivative
        errD = ((deltaError1 pidState') + (deltaError2 pidState') + deltaErr)

    in (kp * err + ki * errI + kd * errD, 

       (FullPidState (errorIntegral pidState')
                     deltaErr  
                     (deltaError1 pidState') 
                     err))
