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

    FullPidState { errorIntegral :: Serial Double,
                   deltaError1 :: Serial Double,
                   deltaError2 :: Serial Double,
                   errorPrev :: Serial Double }

initFullPidState :: FullPidState

initFullPidState = FullPidState 0 0 0 0

computeDemand :: Serial Double ->
                 Serial Double ->
                 Serial Double ->
                 Serial Double ->
                 Serial Double ->
                 FullPidState -> 
                 Serial Double ->
                 Serial Double ->
                 (Serial Double, FullPidState)

computeDemand kp ki kd windupMax valueMax pidState demand value =

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
