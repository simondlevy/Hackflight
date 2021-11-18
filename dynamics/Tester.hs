module Tester where

import Language.Copilot
import Copilot.Compile.C99

import Dynamics
import State
import Utils
import Motors

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
            15000   -- maxrpm

wparams = WorldParams

            9.80665  -- g graviational constant
            1.225    -- rho air density 

fpparams = FixedPitchParams

            5e-06   -- b thrust coefficient [F=b*w^2]
            0.350   -- l arm length [m]

motors = Quad stream_m1 stream_m2 stream_m3 stream_m4

spec = do

    let (state, val) = dynamics wparams vparams fpparams motors time agl

    trigger "stream_debug" true [  arg $ x state
                                 , arg $ y state
                                 , arg $ z state
                                 , arg $ phi state
                                 , arg $ theta state
                                 , arg $ psi state
                                ]
stream_m1 :: SFloat
stream_m1 = extern "stream_m1" Nothing

stream_m2 :: SFloat
stream_m2 = extern "stream_m2" Nothing

stream_m3 :: SFloat
stream_m3 = extern "stream_m3" Nothing

stream_m4 :: SFloat
stream_m4 = extern "stream_m4" Nothing

time :: SFloat
time = extern "stream_time" Nothing

agl :: SFloat
agl = extern "stream_agl" Nothing

main = reify spec >>= compile "haskell"
