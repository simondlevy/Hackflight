{--
  PID control

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module PidControl where

import State
import Demands

data AltHoldState = AltHoldState { errorI :: Double, timePrev :: Double } deriving (Show)

type AltHoldFun = Time -> VehicleState -> Demands -> AltHoldState -> (Demands, AltHoldState)

newAltHoldController :: Double -> Double -> Double -> Double -> (AltHoldFun, AltHoldState)
newAltHoldController target kp ki windupMax = 
    ((altHoldClosure target kp ki windupMax), (AltHoldState 0 0))

errorIntegral :: AltHoldState -> Double
errorIntegral (AltHoldState e _) = e

previousTime :: AltHoldState -> Double
previousTime (AltHoldState _ t) = t

altHoldClosure :: Double -> Double -> Double -> Double -> AltHoldFun
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

