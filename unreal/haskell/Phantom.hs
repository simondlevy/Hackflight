{--
  Haskell Copilot support DJI Phantom simulation

  Copyright(C) 2021 on D.Levy

  MIT License
--}

module Phantom where

import Language.Copilot
import Copilot.Compile.C99

-- Core
import HackflightSim
import State
import Demands
import Receiver
import Time
import Mixers
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
           , posHoldController 0.1  -- Kp
          ]

vparams = VehicleParams

            -- Estimated
            2e-06 -- d drag cofficient [T=d*w^2]

            -- https:--www.dji.com/phantom-4/info
            1.380  -- m mass [kg]

            -- Estimated
            2      -- Ix [kg*m^2] 
            2      -- Iy [kg*m^2] 
            3      -- Iz [kg*m^2] 
            38e-04 -- Jr prop inertial [kg*m^2] 
            15000-- maxrpm

wparams = WorldParams

            9.80665  -- g graviational constant
            1.225    -- rho air density 

fpparams = FixedPitchParams

            5e-06   -- b thrust coefficient [F=b*w^2]
            0.350   -- l arm length [m]

------------------------------------------------------------

spec = do

  let motors = hackflight receiver wparams vparams fpparams pidfuns stream_time quadxap

  -- Call some C routines for getting receiver demands
  trigger "stream_getReceiverDemands" true []

  -- Send the motor values using the external C function
  trigger "stream_writeMotors" true [  arg $ m1 motors
                                     , arg $ m2 motors
                                     , arg $ m3 motors
                                     , arg $ m4 motors ]

------------------------------------------------------------

stream_time :: SFloat
stream_time = extern "stream_time" Nothing

------------------------------------------------------------

main = reify spec >>= compile "hackflight"
