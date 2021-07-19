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
                   previousTime ::  Double,
                   target :: Double,
                   inBand :: Bool }

type PidFun = Time -> VehicleState -> Demands -> PidState 
              -> (Demands, PidState)

data PidController = PidController { pidFun :: PidFun, pidState :: PidState }

newPidController :: PidFun -> PidState -> PidController
newPidController f s = PidController f s

----------------------------- Altitude hold ----------------------------------

newAltHoldController :: Double -> Double -> Double -> Double -> PidController
newAltHoldController kp ki windupMax stickDeadband = 
    PidController (altHoldClosure kp ki windupMax stickDeadband)
                  (AltHoldState 0 0 0 False)

altHoldClosure :: Double -> Double -> Double -> Double -> PidFun
altHoldClosure kp ki windupMax stickDeadband =

    \time -> \vehicleState -> \demands -> \controllerState ->

    let  
         errorI = errorIntegral controllerState
         targetAltitude = target controllerState
 
         -- NED => ENU
         altitude = -(state_z vehicleState)

         throttleDemand = throttle demands

         inband = in_band throttleDemand stickDeadband

         movedInBand = inband && not (inBand controllerState)

         -- Get vehicle state, negating for NED
         z = -(state_z vehicleState)
         dzdt = -(state_dz vehicleState)

         -- Compute dzdt setpoint and error
         dzdt_error = ((target controllerState) - z) - dzdt

         -- Update error integral
         dt = time - (previousTime controllerState)
         newErrorIntegral = constrain_abs((errorIntegral controllerState) +
                                         dzdt_error * dt) windupMax

    -- Compute throttle demand, constrained to [0,1]
    in ((Demands (min (kp * dzdt_error + ki * newErrorIntegral) 1)
                 (roll demands)
                 (pitch demands)
                 (yaw demands)),
        (AltHoldState time newErrorIntegral 0 False))


--------------------------------- Helpers --------------------------------------

constrain_abs :: Double -> Double -> Double
constrain_abs x lim = if x < -lim then -lim else (if x > lim then lim else x)

in_band :: Double -> Double -> Bool
in_band value band = abs(value) < band
