{--
  Altitude-hold PID Controller

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module AltHoldPid where


import State
import Demands

data AltHoldState = AltHoldState { previousTime :: Double , errorIntegral :: Double } deriving (Show)

newAltHoldState :: AltHoldState
newAltHoldState = AltHoldState 0 0

type AltHoldPid = Time -> VehicleState -> Demands -> AltHoldState -> (Demands, AltHoldState)

data AltHoldPair = AltHoldPair AltHoldPid AltHoldState

altHoldClosure :: Double -> Double -> Double -> Double -> AltHoldPid

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

constrainAbs :: Double -> Double -> Double
constrainAbs x lim = if x < -lim then -lim else (if x > lim then lim else x)

