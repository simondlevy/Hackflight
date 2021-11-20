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

data Mixer = Mixer { motorfun :: MotorFun
                   , rspins :: Spins
                   , pspins :: Spins
                   , yspins :: Spins
                   }

data NewMixer = NewQuadXAP | NewQuadXMW

rps :: SFloat -> SFloat -> SFloat
rps motorval maxrpm = motorval * maxrpm * pi / 30

getRPS :: Motors -> NewMixer -> SFloat -> Motors
getRPS motors NewQuadXAP maxrpm = Quad r1 r2 r3 r4 where
  r1 = rps (m1 motors) maxrpm
  r2 = rps (m2 motors) maxrpm
  r3 = rps (m3 motors) maxrpm
  r4 = rps (m4 motors) maxrpm

thrust :: SFloat -> SFloat -> SFloat
thrust rpsval rho = rho * rpsval**2

getThrusts :: Motors -> NewMixer -> SFloat -> Motors
getThrusts motors NewQuadXAP rho = Quad t1 t2 t3 t4 where
  t1 = thrust (m1 motors) rho
  t2 = thrust (m2 motors) rho
  t3 = thrust (m3 motors) rho
  t4 = thrust (m4 motors) rho

dmds :: Demands -> (SFloat, SFloat, SFloat, SFloat)

getTorque :: Motors -> NewMixer -> SFloat
getTorque motors NewQuadXAP = (m1 motors) + (m2 motors) - (m3 motors) - (m4 motors)

getThrust :: Motors -> NewMixer -> SFloat
getThrust motors NewQuadXAP = (m1 motors) + (m2 motors) + (m3 motors) + (m4 motors)

getRoll :: Motors -> NewMixer -> SFloat
getRoll motors NewQuadXAP = (m2 motors) + (m3 motors) - (m1 motors) - (m4 motors)

getPitch :: Motors -> NewMixer -> SFloat
getPitch motors NewQuadXAP = (m2 motors) + (m4 motors) - (m1 motors) - (m3 motors)

getYaw :: Motors -> NewMixer -> SFloat
getYaw motors NewQuadXAP = (m1 motors) + (m2 motors) - (m3 motors) - (m4 motors)

-- Map throttle demand from [-1,+1] to [0,1]
dmds demands = (((throttle demands) + 1) / 2, roll demands, pitch demands, yaw demands )

newmix :: SafetyFun -> Demands -> NewMixer -> Motors
newmix sfun demands NewQuadXAP = Quad m1 m2 m3 m4 where

  (t, r, p, y) = dmds demands

  m1 = sfun $ t - r - p + y
  m2 = sfun $ t + r + p + y
  m3 = sfun $ t + r - p - y
  m4 = sfun $ t - r + p - y
 
mix sfun demands mixer = Quad m1 m2 m3 m4 where

  (t, r, p, y) = dmds demands

  rs = rspins mixer
  ps = pspins mixer
  ys = yspins mixer

  m1 = sfun $ t  + (s1 rs) * r + (s1 ps) * p + (s1 ys) * y
  m2 = sfun $ t  + (s2 rs) * r + (s2 ps) * p + (s2 ys) * y
  m3 = sfun $ t  + (s3 rs) * r + (s3 ps) * p + (s3 ys) * y
  m4 = sfun $ t  + (s4 rs) * r + (s4 ps) * p + (s4 ys) * y

quadxmw = Mixer quadfun rspins pspins yspins where

    rspins = Spins (-1) (-1) (1) (1)
    pspins = Spins (1)  (-1) (1) (-1)
    yspins = Spins (-1) (1)  (1) (-1)

quadxap = Mixer quadfun rspins pspins yspins where

    rspins = Spins (-1) (1) (1)  (-1)
    pspins = Spins (-1) (1) (-1) (1)
    yspins = Spins (1)  (1) (-1) (-1)

