{--
  Classic PID controller

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}
{-# LANGUAGE DataKinds        #-}

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

computeDemand :: Stream Double ->
                 Stream Double ->
                 Stream Double ->
                 Stream Double ->
                 Stream Double ->
                 FullPidState -> 
                 Stream Double ->
                 Stream Double ->
                 (Stream Double, FullPidState)

computeDemand kp ki kd windupMax valueMax pidState demand value =

   (0, FullPidState 0 0 0 0)

{--
    let 

        --  Reset PID state on quick value change
        pidState' = if abs(value) > valueMax then initFullPidState else pidState

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

--}
