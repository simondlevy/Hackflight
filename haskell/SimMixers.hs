{--
  Simulated Mixer types

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module SimMixers where

import Language.Copilot

import Demands
import Motors
import Mixers
import Utils

data SimMixer =  SimMixer {  mixer :: Mixer
                           , param :: SFloat
                          }

rps :: SFloat -> SFloat -> SFloat
rps motorval maxrpm = motorval * maxrpm * pi / 30

thrust :: SFloat -> SFloat -> SFloat
thrust rpsval rho = rho * rpsval**2

-- QuadXAP -------------------------------------------------------------------------------

getRPS :: Motors -> SimMixer -> SFloat -> Motors
getRPS motors (SimMixer QuadXAP _) maxrpm = Quad r1 r2 r3 r4 where
  r1 = rps (m1 motors) maxrpm
  r2 = rps (m2 motors) maxrpm
  r3 = rps (m3 motors) maxrpm
  r4 = rps (m4 motors) maxrpm

getThrusts :: Motors -> SimMixer -> SFloat -> Motors
getThrusts motors (SimMixer QuadXAP _) rho = Quad t1 t2 t3 t4 where
  t1 = thrust (m1 motors) rho
  t2 = thrust (m2 motors) rho
  t3 = thrust (m3 motors) rho
  t4 = thrust (m4 motors) rho

getTorque :: Motors -> SimMixer -> SFloat
getTorque motors (SimMixer QuadXAP _) =
  (m1 motors) + (m2 motors) - (m3 motors) - (m4 motors)

getThrust :: Motors -> SimMixer -> SFloat
getThrust motors (SimMixer QuadXAP _) =
  (m1 motors) + (m2 motors) + (m3 motors) + (m4 motors)

getRoll :: Motors -> SimMixer -> SFloat
getRoll motors (SimMixer QuadXAP l) =
  l * ((m2 motors) + (m3 motors) - (m1 motors) - (m4 motors))

getPitch :: Motors -> SimMixer -> SFloat
getPitch motors (SimMixer QuadXAP l) =
  l * ((m2 motors) + (m4 motors) - (m1 motors) - (m3 motors))

getYaw :: Motors -> SimMixer -> SFloat
getYaw motors (SimMixer QuadXAP _) =
  (m1 motors) + (m2 motors) - (m3 motors) - (m4 motors)

-- ThrustVector --------------------------------------------------------------------------

