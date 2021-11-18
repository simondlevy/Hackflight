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

mval = 1.0

motors = Quad mval mval mval mval

spec = do

    let (state, val) = dynamics wparams vparams fpparams motors time agl

    trigger "stream_debug" true [arg $ z state]

time :: SFloat
time = extern "stream_time" Nothing

agl :: SFloat
agl = extern "stream_agl" Nothing

main = reify spec >>= compile "haskell"
