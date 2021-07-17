{--
  PID control

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module PidControl where

import State
import Demands

type PidControllerState = [Double]

type PidControllerFun = Time -> VehicleState -> Demands -> PidControllerState ->(Demands, PidControllerState)

--------------------------------------------------------------------------------

data AltHoldState = AltHoldState Double Double

errorI :: AltHoldState -> Double
errorI (AltHoldState e _) = e

prevTime :: AltHoldState -> Double
prevTime (AltHoldState _ t) = t

newAltHoldController :: Double -> Double -> Double -> Double -> (PidControllerFun, PidControllerState)
newAltHoldController target kp ki windupMax = 
    ((altHoldClosure target kp ki windupMax), [0,0])

errorIntegral :: PidControllerState -> Double
errorIntegral controllerState = controllerState!!0

previousTime :: PidControllerState -> Double
previousTime controllerState = controllerState!!1

altHoldClosure :: Double -> Double -> Double -> Double -> PidControllerFun
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

    in ((Demands u 0 0 0), [time, newErrorIntegral])

--------------------------------------------------------------------------------

constrainAbs :: Double -> Double -> Double
constrainAbs x lim = if x < -lim then -lim else (if x > lim then lim else x)

