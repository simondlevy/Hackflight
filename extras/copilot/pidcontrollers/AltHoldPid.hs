{--
  PID controller for altitude hold

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}
{-# LANGUAGE DataKinds        #-}

-- jmodule AltHoldPid(altHoldController)
module AltHoldPid

where

import Language.Copilot
import Copilot.Compile.C99

import VehicleState
import PidControllers
import Demands
import Utils(constrain_abs, in_band)

{--
altHoldController ::    Stream Double
                     -> Stream Double
                     -> Stream Double
                     -> Stream Double
                     -> Stream Double 
                     -> PidController

altHoldController kp ki windupMax pilotVelZMax stickDeadband = 
    makePidController (altHoldFun kp ki windupMax pilotVelZMax stickDeadband)
                      (AltHoldState 0 0 False)


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
         controllerState' = if inband && not (altInBand controllerState)
                            then AltHoldState 0 altitude True
                            else controllerState

         -- Target velocity is a setpoint inside deadband, scaled
         -- constant outside
         altTargetVelocity = if inband
                             then (altTarget controllerState') - altitude
                             else pilotVelZMax * throttleDemand


         -- Compute error as altTarget velocity minus actual velocity, after
         -- negating actual to accommodate NED
         err = altTargetVelocity + (dz vehicleState)

         -- Accumualte error integral
         errI = constrain_abs ((altErrorIntegral controllerState) + err)
                windupMax

    -- Return updated demands and controller state
    in  (Demands (err * kp + errI * ki) 0 0 0,
         AltHoldState errI (altTarget controllerState') inband)
--}
