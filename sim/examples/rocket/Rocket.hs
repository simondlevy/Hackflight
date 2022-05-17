{--
  Haskell Copilot support for thrust-vector rocket simulation

  Copyright(C) 2021 on D.Levy

  MIT License
--}

module Rocket where

import Language.Copilot
import Copilot.Compile.C99

-- Core
import HackflightSim
import State
import Demands
import Receiver
import Time
import Mixers
import ThrustVector
import Motors
import Dynamics
import Utils

-- PID controllers
import PidController
import RatePid(rateController)
import YawPid(yawController)
import LevelPid(levelController)
import AltHoldPid(altHoldController)
import PosHoldPid(posHoldController)

------------------------------------------------------------

receiver = makeReceiver 4.0 -- demand scale

pidfuns = [  
             altHoldController 0.75 1.5   -- Kp, Ki
           , levelController 0.2 -- Kp
           , yawController 1.0625 0.005625 -- Kp, Ki
           , rateController 0.225  0.001875 0.375 -- Kp, Ki, Kd 
           -- , posHoldController 0.1  -- Kp
          ]

vparams = VehicleParams

            -- Estimated
            2e-06 -- d drag cofficient [T=d*w^2]

            -- https:--www.dji.com/phantom-4/info
            1.380  -- m mass [kg]

            -- Estimated
            2       -- Ix [kg*m^2] 
            2       -- Iy [kg*m^2] 
            3       -- Iz [kg*m^2] 
            3.8e-03 -- Jr prop inertial [kg*m^2] 
            5e-06   -- b thrust coefficient [F=b*w^2] ??? XXX ???
            15000   -- maxrpm

wparams = WorldParams

            9.80665  -- g graviational constant
            1.225    -- rho air density 

nozzleAngle = 45.0 -- [deg]

------------------------------------------------------------

spec = do

  let (state, motors) = hackflight receiver
                                   wparams
                                   vparams
                                   pidfuns
                                   (thrustVectorDynamics nozzleAngle)
                                   stream_time 
                                   stream_agl

  -- Call some C routines for getting receiver demands
  trigger "stream_getReceiverDemands" true []

  -- trigger "stream_debug" true [ arg val ]

  -- Set the motor values using the external C function
  trigger "stream_setMotorsQuad" true [  arg $ m1 motors
                                       , arg $ m2 motors
                                       , arg $ m3 motors
                                       , arg $ m4 motors ]

  -- Send the vehicle pose using the external C function
  trigger "stream_setPose" true [  arg $ x state
                                 , arg $ y state
                                 , arg $ z state
                                 , arg $ phi state
                                 , arg $ theta state
                                 , arg $ psi state ]

------------------------------------------------------------

stream_time :: SFloat
stream_time = extern "stream_time" Nothing

stream_agl :: SFloat
stream_agl = extern "stream_agl" Nothing

------------------------------------------------------------

main = reify spec >>= compile "hackflight"
