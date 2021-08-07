{--
  PID control for roll/pitch angular velocity

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module RatePid(rateController)

where

import VehicleState
import PidControl
import Demands
import Utils

rateController :: Double -> Double -> Double -> Double -> Double ->
                  PidController

rateController kp ki kd windupMax rateMax = 
    PidController (rateFun kp ki kd windupMax rateMax)
                  (RateState (FullPidState 0 0 0 0) (FullPidState 0 0 0 0))

rateFun :: Double -> Double -> Double -> Double -> Double -> PidFun
rateFun kp ki kd windupMax rateMax vehicleState demands controllerState =

    let 

        computeDof demand angularVelocity oldPidState =

            let 

                --  Reset PID state on quick angular velocity change
                newPidState = if abs(angularVelocity) > deg2rad(rateMax)
                              then (FullPidState 0 0 0 0)
                              else oldPidState

                err = demand - angularVelocity

                errI = constrain_abs ((fullErrorIntegral newPidState) + err)
                                      windupMax
                deltaErr = err - (fullErrorPrev newPidState)
                errD = ((fullDeltaError1 newPidState) +
                        (fullDeltaError2 newPidState) +
                        deltaErr)

                in ((kp * err) + (ki * errI) + (kd * errD), 
                    (FullPidState (fullErrorIntegral newPidState)
                                  deltaErr  
                                  (fullDeltaError1 newPidState) 
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
