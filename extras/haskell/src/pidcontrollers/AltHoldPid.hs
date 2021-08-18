{--
  PID controller for altitude hold

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module AltHoldPid(altHoldController)

where

import VehicleState
import PidControllers
import Demands
import Utils(constrain_abs, in_band)

altHoldController :: Double -> Double -> Double -> Double -> Double 
                     -> PidController

altHoldController kp ki windupMax pilotVelZMax stickDeadband = 
    makePidController (altHoldFun kp ki windupMax pilotVelZMax stickDeadband)
                      (AltHoldState 0 0 False)


altHoldFun :: Double -> Double -> Double -> Double -> Double -> PidFun

altHoldFun kp
           ki
           windupMax
           pilotVelZMax
           stickDeadband
           vehicleState
           demands
           controllerState =

    let  
         -- NED => ENU
         altitude = -(z vehicleState)

         throttleDemand = throttle demands

         inband = in_band throttleDemand stickDeadband

         -- Reset controller when moving into deadband
         altTarget' = if inband && not (altInBand controllerState)
                      then altitude
                      else altTarget controllerState

         -- constant outside
         altTargetVelocity = if inband
                             then altTarget' - altitude
                             else pilotVelZMax * throttleDemand

         -- Compute error as altTarget velocity minus actual velocity, after
         -- negating actual to accommodate NED
         err = altTargetVelocity + (dz vehicleState)

         -- Accumualte error integral
         errI = constrain_abs ((altErrorIntegral controllerState) + err) windupMax

    -- Return updated demands and controller state
    in  (Demands (err * kp + errI * ki) 0 0 0, AltHoldState errI altTarget' inband)
