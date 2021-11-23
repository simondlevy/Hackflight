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

data Mixer =  QuadXAP | QuadXMW | ThrustVector | Coaxial

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

mix sfun demands ThrustVector = Quad r1 r2 s1 s2 where

  (t, r, p, y) = dmds demands

  r1 = sfun $ t + y
  r2 = sfun $ t - y
  s1 = r
  s2 = p


-- XXX faking up coaxial mixer with QuadXAP mixer
mix sfun demands Coaxial = mix sfun demands QuadXAP
