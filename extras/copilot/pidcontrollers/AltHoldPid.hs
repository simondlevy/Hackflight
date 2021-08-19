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
import PidController
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
           demands =

    Demands (error' * kp + errorIntegral' * ki) 0 0 0

    where

         altitude = -(z vehicleState)

         throttleDemand = throttle demands

         inband = in_band throttleDemand stickDeadband

         -- Reset controller when moving into deadband
         altitudeTarget' = if inband -- && not (altInBand controllerState)
                           then altitude
                           else altitudeTarget
         altitudeTarget = [0] ++ altitudeTarget'

         targetVelocity = if inband
                             then altitudeTarget' - altitude
                             else pilotVelZMax * throttleDemand

         -- Compute error as altTarget velocity minus actual velocity, after
         -- negating actual to accommodate NED
         error' = targetVelocity + (dz vehicleState)

         -- Accumualte error integral
         errorIntegral' = constrain_abs (errorIntegral + error') windupMax
         errorIntegral = [0] ++ errorIntegral'
