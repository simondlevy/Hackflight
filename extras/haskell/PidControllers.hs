{--
  PID control

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module PidControllers(PidController,
                  newPidController,
                  rateController,
                  yawController,
                  levelController,
                  altHoldController,
                  posHoldController) where

import VehicleState
import PidControl
import Demands
import Utils(constrain_abs, in_band, deg2rad)

-------------------------------------- Rate ----------------------------------

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
                                                 (VehicleState.dphi vehicleState)
                                                 (rateRollState
                                                  controllerState)

        -- Pitch demand is nose-down positive, so we negate pitch-forward
        -- (nose-down negative) to reconcile them
        (pitchDemand, pitchPidControl) = computeDof (Demands.pitch demands)
                                                  (-(VehicleState.dtheta vehicleState))
                                                  (ratePitchState
                                                   controllerState)

    -- Return updated demands and controller state
    in ((Demands (Demands.throttle demands)
                 rollDemand
                 pitchDemand
                 (Demands.yaw demands)),
        (RateState rollPidControl pitchPidControl))

----------------------------------- Yaw ---------------------------------------

yawController :: Double -> Double -> Double -> PidController

yawController kp ki windupMax = 
    PidController (yawClosure kp ki windupMax) (YawState 0)

yawClosure :: Double -> Double -> Double -> PidFun
yawClosure kp ki windupMax =

    \vehicleState -> \demands -> \controllerState ->

    -- Compute error as target minus actual
    let err = (Demands.yaw demands) - (VehicleState.dpsi vehicleState)

        -- Accumualte error integral
        errI = constrain_abs ((yawErrorIntegral controllerState) + err)
                             windupMax

    -- Return updated demands and controller state
    in (Demands (Demands.throttle demands) 
                (Demands.roll demands)
                (Demands.pitch demands)
                (kp * err + ki * errI),
        YawState errI)

---------------------------------- Level --------------------------------------

levelController :: Double -> Double -> PidController

levelController kp maxAngleDegrees =
    PidController (levelClosure kp maxAngleDegrees) NoState

levelClosure :: Double -> Double -> PidFun
levelClosure kp maxAngleDegrees =

    \vehicleState -> \demands -> \_controllerState ->

    let 
        -- Maximum roll pitch demand is +/-0.5, so to convert demand to
        -- angle for error computation, we multiply by the folling amount:
        demandScale = 2 * deg2rad(maxAngleDegrees)

    -- Return updated demands and controller state
    in (Demands (Demands.throttle demands) 

                -- New roll demand
                (kp * (Demands.roll demands) * demandScale -
                      (VehicleState.phi vehicleState))

                -- Pitch demand is nose-down positive, so we negate
                -- pitch-forward (nose-down negative) to reconcile them
                -- and get new pitch demand
                (kp * (Demands.pitch demands) * demandScale +
                      (VehicleState.theta vehicleState))

                (Demands.yaw demands),

        NoState)

----------------------------- Altitude hold -----------------------------------

altHoldController :: Double -> Double -> Double -> Double -> Double 
                        -> PidController
altHoldController kp ki windupMax pilotVelZMax stickDeadband = 
    PidController (altHoldClosure kp ki windupMax pilotVelZMax stickDeadband)
                  (AltHoldState 0 0 False)

altHoldClosure :: Double -> Double -> Double -> Double -> Double -> PidFun
altHoldClosure kp ki windupMax pilotVelZMax stickDeadband =

    \vehicleState -> \demands -> \controllerState ->

    let  
         -- NED => ENU
         altitude = -(VehicleState.z vehicleState)

         throttleDemand = throttle demands

         inband = in_band throttleDemand stickDeadband

         -- Reset controller when moving into deadband
         newControllerState = if inband && not (altInBand controllerState)
                              then AltHoldState 0 altitude True
                              else controllerState

         -- Target velocity is a setpoint inside deadband, scaled
         -- constant outside
         altTargetVelocity = if inband
                          then (altTarget newControllerState) - altitude
                          else pilotVelZMax * throttleDemand


         -- Compute error as altTarget velocity minus actual velocity, after
         -- negating actual to accommodate NED
         err = altTargetVelocity + (VehicleState.dz vehicleState)

         -- Accumualte error integral
         errI = constrain_abs ((altErrorIntegral controllerState) + err)
                windupMax

    -- Return updated demands and controller state
    in  (Demands (err * kp + errI * ki)
                 (Demands.roll demands)
                 (Demands.pitch demands)
                 (Demands.yaw demands),
         AltHoldState errI (altTarget newControllerState) inband)
                      

---------------------------- Position hold ------------------------------------

posHoldController :: Double -> Double -> PidController

posHoldController kp stickDeadband =
    PidController (posHoldClosure kp stickDeadband) NoState

posHoldClosure :: Double -> Double -> PidFun
posHoldClosure kp stickDeadband =

    \vehicleState -> \demands -> \_controllerState ->

    let newDemands = if in_band (Demands.roll demands) stickDeadband &&
                        in_band (Demands.pitch demands) stickDeadband
        
                    then

                        let p = VehicleState.psi vehicleState

                            -- Rotate X, Y velocities into body frame

                            cp = cos p
                            sp = sin p

                            xx = VehicleState.dx vehicleState
                            yy = VehicleState.dy vehicleState

                        in Demands (Demands.throttle demands)
                                   (-kp * (cp * xx + sp * yy))
                                   (-kp * (cp * yy - sp * xx))
                                   (Demands.yaw demands)

                    else demands

    in (newDemands, NoState)
