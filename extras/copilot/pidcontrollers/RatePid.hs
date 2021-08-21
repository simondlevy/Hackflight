{--
  PID control for roll/pitch angular velocity

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module RatePid(rateController)

where

import VehicleState
import PidController
import FullPidController
import Demands

rateController :: Double -> Double -> Double -> Double -> Double ->
                  PidController

rateController kp ki kd windupMax rateMax = 
    makePidController (rateFun kp ki kd windupMax rateMax)
                      (RateState initFullPidState initFullPidState)


rateFun :: Double -> Double -> Double -> Double -> Double -> PidFun

rateFun kp ki kd windupMax rateMax vehicleState demands pidState' =

    let 

        computeDemand' pfun dfun vfun =
            computeDemand kp
                          ki
                          kd
                          windupMax
                          rateMax
                          (pfun pidState')
                          (dfun demands)
                          (vfun vehicleState)

        (rollDemand, rollState) =
            computeDemand' rateRollState roll dphi

        -- Pitch demand is nose-down positive, so we negate pitch-forward
        -- (nose-down negative) to reconcile them
        (pitchDemand, pitchState) =
            computeDemand' ratePitchState pitch (\vs -> -(dtheta vs))

    -- Return updated demands and controller state
    in (Demands 0 rollDemand pitchDemand 0, RateState rollState pitchState)
