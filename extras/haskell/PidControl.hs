{--
  PID control

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module PidControl(PidController,
                  pidFun,
                  pidState,
                  newPidController,
                  newRateController,
                  newAltHoldController) where

import State
import Demands

data FullPidState = 

    FullPidState { fullErrorIntegral :: Double,
                   fullDeltaError1 :: Double,
                   fullDeltaError2 :: Double,
                   fullErrorPrev :: Double }

data PidState =

     AltHoldState { altErrorIntegral :: Double,
                    altTarget :: Double,
                    altInBand :: Bool }

   | RateState { rateRollState :: FullPidState,
                 ratePitchState :: FullPidState }

   | YawState { yawErrorIntegral :: Double }

type PidFun = VehicleState -> Demands -> PidState -> (Demands, PidState)

data PidController = PidController { pidFun :: PidFun, pidState :: PidState }

newPidController :: PidFun -> PidState -> PidController
newPidController f s = PidController f s

-------------------------------------- Rate ----------------------------------

newRateController :: Double -> Double -> Double -> Double -> Double ->
                     PidController

newRateController kp ki kd windupMax rateMax = 
    PidController (rateClosure kp ki kd windupMax rateMax)
                  (RateState (FullPidState 0 0 0 0) (FullPidState 0 0 0 0))

rateClosure :: Double -> Double -> Double -> Double -> Double -> PidFun
rateClosure kp ki kd windupMax rateMax =

    \vehicleState -> \demands -> \controllerState ->

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

        (rollDemand, rollPidState) = computeDof (roll demands)
                                                 (state_dphi vehicleState)
                                                 (rateRollState
                                                  controllerState)

        -- Pitch demand is nose-down positive, so we negate pitch-forward
        -- (nose-down negative) to reconcile them
        (pitchDemand, pitchPidState) = computeDof (pitch demands)
                                                  (-(state_dtheta vehicleState))
                                                  (ratePitchState
                                                   controllerState)

    -- Return updated demands and controller state
    in ((Demands (throttle demands) rollDemand pitchDemand (yaw demands)),
        (RateState rollPidState pitchPidState))

----------------------------------- Yaw ---------------------------------------

newYawController :: Double -> Double -> Double -> PidController

newYawController kp ki windupMax = 
    PidController (yawClosure kp ki windupMax) (YawState 0)

yawClosure :: Double -> Double -> Double -> PidFun
yawClosure kp ki windupMax =

    \vehicleState -> \demands -> \controllerState ->

    -- Compute error as target minus actual
    let err = (yaw demands) - state_dpsi vehicleState

        -- Accumualte error integral
        errI = constrain_abs ((yawErrorIntegral controllerState) + err)
                             windupMax

    -- Return updated demands and controller state
    in (Demands (throttle demands) 
                (roll demands)
                (pitch demands)
                (kp * err + ki * errI),
        YawState errI)

----------------------------- Altitude hold -----------------------------------

newAltHoldController :: Double -> Double -> Double -> Double -> Double 
                        -> PidController
newAltHoldController kp ki windupMax pilotVelZMax stickDeadband = 
    PidController (altHoldClosure kp ki windupMax pilotVelZMax stickDeadband)
                  (AltHoldState 0 0 False)

altHoldClosure :: Double -> Double -> Double -> Double -> Double -> PidFun
altHoldClosure kp ki windupMax pilotVelZMax stickDeadband =

    \vehicleState -> \demands -> \controllerState ->

    let  
         -- NED => ENU
         altitude = -(state_z vehicleState)

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
         err = altTargetVelocity + (state_dz vehicleState)

         -- Accumualte error integral
         errI = constrain_abs ((altErrorIntegral controllerState) + err)
                windupMax

    -- Return updated demands and controller state
    in  (Demands (err * kp + errI * ki)
                 (roll demands)
                 (pitch demands)
                 (yaw demands),
         AltHoldState errI (altTarget newControllerState) inband)
                      

--------------------------------- Helpers --------------------------------------

constrain_abs :: Double -> Double -> Double
constrain_abs x lim = if x < -lim then -lim else (if x > lim then lim else x)

in_band :: Double -> Double -> Bool
in_band value band = abs(value) < band

deg2rad :: Double -> Double
deg2rad d = d * pi / 180
