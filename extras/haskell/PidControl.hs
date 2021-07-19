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
                   inBand :: Bool }

type PidFun = Time -> VehicleState -> Demands -> PidState 
              -> (Demands, PidState)

data PidController = PidController { pidFun :: PidFun, pidState :: PidState }

newPidController :: PidFun -> PidState -> PidController
newPidController f s = PidController f s

----------------------------- Altitude hold ----------------------------------

newAltHoldController :: Double -> Double -> Double -> PidController
newAltHoldController kp ki windupMax = 
    PidController (altHoldClosure kp ki windupMax) (AltHoldState 0 0 False)

altHoldClosure :: Double -> Double -> Double -> PidFun
altHoldClosure kp ki windupMax  =

    \time -> \vehicleState -> \demands -> \controllerState ->

    let  

         -- Get vehicle state, negating for NED
         z = -(state_z vehicleState)
         dzdt = -(state_dz vehicleState)

         -- Compute dzdt setpoint and error
         dzdt_error = (0 - z) - dzdt

         -- Update error integral
         dt = time - (previousTime controllerState)
         newErrorIntegral =constrainAbs((errorIntegral controllerState) +
                                        dzdt_error * dt) windupMax

    -- Compute throttle demand, constrained to [0,1]
    in ((Demands (min (kp * dzdt_error + ki * newErrorIntegral) 1)
                 (roll demands)
                 (pitch demands)
                 (yaw demands)),
        (AltHoldState time newErrorIntegral False))


--------------------------------- Helpers --------------------------------------

constrainAbs :: Double -> Double -> Double
constrainAbs x lim = if x < -lim then -lim else (if x > lim then lim else x)
