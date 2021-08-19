{--
  PID controller for altitude hold

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}
{-# LANGUAGE DataKinds        #-}

module AltHoldPid

where

import Language.Copilot
import Copilot.Compile.C99

import VehicleState
import PidControllers
import Demands
import Utils(constrain_abs, in_band)

altHoldController ::    Stream Double
                     -> Stream Double
                     -> Stream Double
                     -> Stream Double
                     -> Stream Double 
                     -> PidController

altHoldController kp ki windupMax pilotVelZMax stickDeadband = 
    makePidController (altHoldFun kp ki windupMax pilotVelZMax stickDeadband)
                      (AltHoldState 0 0 false)


altHoldFun ::    Stream Double
              -> Stream Double
              -> Stream Double
              -> Stream Double
              -> Stream Double
              -> PidFun

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

         altTargetVelocity = if inband
                             then altTarget' - altitude
                             else pilotVelZMax * throttleDemand

         -- Compute error as altTarget velocity minus actual velocity, after
         -- negating actual to accommodate NED
         error' = altTargetVelocity + (dz vehicleState)

         -- Accumualte error integral
         errorIntegral = constrain_abs ((altErrorIntegral controllerState) + error')
                         windupMax

    -- Return updated demands and controller state
    in  (Demands (error' * kp + errorIntegral * ki) 0 0 0,
         AltHoldState errorIntegral altTarget' inband)
