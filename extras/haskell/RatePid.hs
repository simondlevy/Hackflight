{--
  PID control for pitch/level angular velocity

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module RatePid(rateController)

where

import VehicleState
import PidControl
import Demands
import Utils(constrain_abs, deg2rad)

rateController :: Double -> Double -> Double -> Double -> Double ->
                  PidController

rateController kp ki kd windupMax rateMax = 
    PidController (rateClosure kp ki kd windupMax rateMax)
                  (RateState (FullPidControl 0 0 0 0) (FullPidControl 0 0 0 0))

rateClosure :: Double -> Double -> Double -> Double -> Double -> PidFun
rateClosure kp ki kd windupMax rateMax =

    \vehicleState -> \demands -> \controllerState ->

    let 

        computeDof demand angularVelocity oldPidControl =

            let 

                --  Reset PID state on quick angular velocity change
                newPidControl = if abs(angularVelocity) > deg2rad(rateMax)
                              then (FullPidControl 0 0 0 0)
                              else oldPidControl

                err = demand - angularVelocity

                errI = constrain_abs ((fullErrorIntegral newPidControl) + err)
                                      windupMax
                deltaErr = err - (fullErrorPrev newPidControl)
                errD = ((fullDeltaError1 newPidControl) +
                        (fullDeltaError2 newPidControl) +
                        deltaErr)

                in ((kp * err) + (ki * errI) + (kd * errD), 
                    (FullPidControl (fullErrorIntegral newPidControl)
                                  deltaErr  
                                  (fullDeltaError1 newPidControl) 
                                  err))

        (rollDemand, rollPidControl) = computeDof (Demands.roll demands)
                                                 (VehicleState.dphi
                                                  vehicleState)
                                                 (rateRollState
                                                  controllerState)

        -- Pitch demand is nose-down positive, so we negate pitch-forward
        -- (nose-down negative) to reconcile them
        (pitchDemand, pitchPidControl) = computeDof (Demands.pitch demands)
                                                  (-(VehicleState.dtheta
                                                     vehicleState))
                                                  (ratePitchState
                                                   controllerState)

    -- Return updated demands and controller state
    in ((Demands (Demands.throttle demands)
                 rollDemand
                 pitchDemand
                 (Demands.yaw demands)),
        (RateState rollPidControl pitchPidControl))
