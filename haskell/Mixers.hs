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

type MixerFun = SafetyFun -> Demands -> Motors

data Mixer = Mixer { mixerfun :: MixerFun, motorfun :: MotorFun }

dmds :: Demands -> (SFloat, SFloat, SFloat, SFloat)

-- Map throttle demand from [-1,+1] to [0,1]
dmds demands = (((throttle demands) + 1) / 2, roll demands, pitch demands, yaw demands )

quadxmwfun :: MixerFun

quadxmwfun sfun demands = QuadMotors m1 m2 m3 m4 where

  (t, r, p, y) = dmds demands

  m1 = sfun $ t - r + p  - y
  m2 = sfun $ t - r - p  + y
  m3 = sfun $ t + r + p  + y
  m4 = sfun $ t + r - p  - y

quadxmw = Mixer quadxmwfun quadFun

quadxapfun :: MixerFun

quadxapfun sfun demands = QuadMotors m1 m2 m3 m4 where

  (t, r, p, y) = dmds demands

  m1 = sfun $ t - r - p  + y
  m2 = sfun $ t + r + p  + y
  m3 = sfun $ t + r - p  - y
  m4 = sfun $ t - r + p  - y

quadxap = Mixer quadxapfun quadFun
