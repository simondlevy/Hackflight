{--
  PID control

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module PidControl(PidFun, PidState, newAltHoldController) where

import State
import Demands

data PidState = AltHoldState Double Double

type PidFun = Time -> VehicleState -> Demands -> PidState -> (Demands, PidState)

newAltHoldController :: Double -> Double -> Double -> Double -> (PidFun, PidState)
newAltHoldController target kp ki windupMax = 
    ((altHoldClosure target kp ki windupMax), (AltHoldState 0 0))

errorIntegral :: PidState -> Double
errorIntegral (AltHoldState e _) = e

previousTime :: PidState -> Double
previousTime (AltHoldState _ t) = t

altHoldClosure :: Double -> Double -> Double -> Double -> PidFun
altHoldClosure target kp ki windupMax  =

    \time -> \vehicleState -> \_demands -> \controllerState ->

    let  
         -- Get vehicle state, negating for NED
         z = -(state_z vehicleState)
         dzdt = -(state_dz vehicleState)

         -- Compute dzdt setpoint and error
         dzdt_error = (target - z) - dzdt

         -- Update error integral
         dt = time - (previousTime controllerState)
         newErrorIntegral = constrainAbs ((errorIntegral controllerState) + dzdt_error * dt) windupMax

         -- Compute throttle demand, constrained to [0,1]
         u = min (kp * dzdt_error + ki * newErrorIntegral) 1

    in ((Demands u 0 0 0), (AltHoldState time newErrorIntegral))

--------------------------------------------------------------------------------

constrainAbs :: Double -> Double -> Double
constrainAbs x lim = if x < -lim then -lim else (if x > lim then lim else x)
