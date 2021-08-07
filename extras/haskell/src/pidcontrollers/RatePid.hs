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

        computeDof demand angularVelocity pidState' =

            let 

                --  Reset PID state on quick angular velocity change
                pidState'' = if abs(angularVelocity) > deg2rad(rateMax)
                            then (FullPidState 0 0 0 0)
                            else pidState'

                err = demand - angularVelocity

                errI = constrain_abs ((fullErrorIntegral pidState'') + err)
                                      windupMax
                deltaErr = err - (fullErrorPrev pidState'')
                errD = ((fullDeltaError1 pidState'') +
                        (fullDeltaError2 pidState'') +
                        deltaErr)

                in ((kp * err) + (ki * errI) + (kd * errD), 
                    (FullPidState (fullErrorIntegral pidState'')
                                  deltaErr  
                                  (fullDeltaError1 pidState'') 
                                  err))

        (rollDemand, rollPidControl) = computeDof (roll demands)
                                                  (dphi vehicleState)
                                                  (rateRollState
                                                  controllerState)

        -- Pitch demand is nose-down positive, so we negate pitch-forward
        -- (nose-down negative) to reconcile them
        (pitchDemand, pitchPidControl) = computeDof (pitch demands)
                                                    (-(dtheta vehicleState))
                                                    (ratePitchState
                                                    controllerState)

    -- Return updated demands and controller state
    in (Demands 0 rollDemand pitchDemand 0,
        RateState rollPidControl pitchPidControl)
