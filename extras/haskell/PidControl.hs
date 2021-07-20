{--
  PID control

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module PidControl(PidController,
                  pidFun,
                  pidState,
                  newPidController,
                  newAltHoldController) where

import State
import Demands

data FullPidState = 

    FullPidState { pidErrorIntegral :: Double,
                   pidDeltaError1 :: Double,
                   pidDeltaError2 :: Double,
                   pidErrorPrev :: Double }

data PidState =

     AltHoldState { altErrorIntegral :: Double,
                    altTarget :: Double,
                    altInBand :: Bool }

   | RateState { rateRollState :: FullPidState,
                 ratePitchState :: FullPidState }

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

    -- Return updated demands and controller state
    (Demands (throttle demands)
             (getDemand demands)
             (getDemand demands)
             (yaw demands),
         RateState (FullPidState 0 0 0 0) (FullPidState 0 0 0 0))

    where getDemand demands = 0


----------------------------- Altitude hold ----------------------------------

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
