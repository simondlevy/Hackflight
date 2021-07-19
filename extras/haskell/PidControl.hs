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

data PidState =
    AltHoldState { errorIntegral :: Double,
                   target :: Double,
                   inBand :: Bool }

type PidFun = VehicleState -> Demands -> PidState -> (Demands, PidState)

data PidController = PidController { pidFun :: PidFun, pidState :: PidState }

newPidController :: PidFun -> PidState -> PidController
newPidController f s = PidController f s

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
         newControllerState = if inband && not (inBand controllerState)
                              then AltHoldState 0 altitude True
                              else controllerState

         -- Target velocity is a setpoint inside deadband, scaled
         -- constant outside
         targetVelocity = if inband
                          then (target newControllerState) - altitude
                          else pilotVelZMax * throttleDemand


         -- Compute error as target velocity minus actual velocity, after
         -- negating actual to accommodate NED
         err = targetVelocity + (state_dz vehicleState)

         -- Accumualte error integral
         errI = constrain_abs ((errorIntegral controllerState) + err) windupMax

    -- Return updated demands and controller state
    in  (Demands (err * kp + errI * ki)
                 (roll demands)
                 (pitch demands)
                 (yaw demands),
         AltHoldState errI (target newControllerState) inband)
                      

--------------------------------- Helpers --------------------------------------

constrain_abs :: Double -> Double -> Double
constrain_abs x lim = if x < -lim then -lim else (if x > lim then lim else x)

in_band :: Double -> Double -> Bool
in_band value band = abs(value) < band
