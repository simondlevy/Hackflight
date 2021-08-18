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

--}

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

    -- Return updated demands and controller state
    in  (Demands 0 0 0 0,
         AltHoldState 0 0 false)
