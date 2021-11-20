{--
  Mixer types

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module Mixers where

import Language.Copilot

import Demands
import Motors
import Utils

type SafetyFun = SFloat -> SFloat

data Mixer = QuadXAP | QuadXMW

motorfun :: Mixer -> MotorFun
motorfun QuadXMW = quadfun
motorfun QuadXAP = quadfun

-- Map throttle demand from [-1,+1] to [0,1]
dmds demands = (((throttle demands) + 1) / 2, roll demands, pitch demands, yaw demands )

mix :: SafetyFun -> Demands -> Mixer -> Motors

mix sfun demands QuadXAP = Quad m1 m2 m3 m4 where

  (t, r, p, y) = dmds demands

  m1 = sfun $ t - r - p + y
  m2 = sfun $ t + r + p + y
  m3 = sfun $ t + r - p - y
  m4 = sfun $ t - r + p - y
 
mix sfun demands QuadXMW = Quad m1 m2 m3 m4 where

  (t, r, p, y) = dmds demands

  m1 = sfun $ t - r + p - y
  m2 = sfun $ t - r - p + y
  m3 = sfun $ t + r + p + y
  m4 = sfun $ t + r - p - y

-- For simulation dynamics --------------------------------------

rps :: SFloat -> SFloat -> SFloat
rps motorval maxrpm = motorval * maxrpm * pi / 30

getRPS :: Motors -> Mixer -> SFloat -> Motors
getRPS motors QuadXAP maxrpm = Quad r1 r2 r3 r4 where
  r1 = rps (m1 motors) maxrpm
  r2 = rps (m2 motors) maxrpm
  r3 = rps (m3 motors) maxrpm
  r4 = rps (m4 motors) maxrpm

thrust :: SFloat -> SFloat -> SFloat
thrust rpsval rho = rho * rpsval**2

getThrusts :: Motors -> Mixer -> SFloat -> Motors
getThrusts motors QuadXAP rho = Quad t1 t2 t3 t4 where
  t1 = thrust (m1 motors) rho
  t2 = thrust (m2 motors) rho
  t3 = thrust (m3 motors) rho
  t4 = thrust (m4 motors) rho

dmds :: Demands -> (SFloat, SFloat, SFloat, SFloat)

getTorque :: Motors -> Mixer -> SFloat
getTorque motors QuadXAP = (m1 motors) + (m2 motors) - (m3 motors) - (m4 motors)

getThrust :: Motors -> Mixer -> SFloat
getThrust motors QuadXAP = (m1 motors) + (m2 motors) + (m3 motors) + (m4 motors)

getRoll :: Motors -> Mixer -> SFloat
getRoll motors QuadXAP = (m2 motors) + (m3 motors) - (m1 motors) - (m4 motors)

getPitch :: Motors -> Mixer -> SFloat
getPitch motors QuadXAP = (m2 motors) + (m4 motors) - (m1 motors) - (m3 motors)

getYaw :: Motors -> Mixer -> SFloat
getYaw motors QuadXAP = (m1 motors) + (m2 motors) - (m3 motors) - (m4 motors)
