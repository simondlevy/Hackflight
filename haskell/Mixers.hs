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

-- quadfun :: SafetyFun -> Demands -> Spins -> Spins -> Spins -> Motors
quadfun :: SafetyFun -> Demands -> Spins -> Spins -> Spins -> Motors

quadfun sfun demands rspins pspins yspins = Quad m1 m2 m3 m4 where

  (t, r, p, y) = dmds demands

  m1 = sfun $ t  + (s1 rspins) * r + (s1 pspins) * p + (s1 yspins) * y
  m2 = sfun $ t  + (s2 rspins) * r + (s2 pspins) * p + (s2 yspins) * y
  m3 = sfun $ t  + (s3 rspins) * r + (s3 pspins) * p + (s3 yspins) * y
  m4 = sfun $ t  + (s4 rspins) * r + (s4 pspins) * p + (s4 yspins) * y

quadxmwfun :: MixerFun

quadxmwfun sfun demands = quadfun sfun demands rspins pspins yspins where

    rspins = Spins (-1) (-1) 1   1
    pspins = Spins   1  (-1) 1 (-1)
    yspins = Spins (-1)   1  1 (-1)

quadxmw = Mixer quadxmwfun quadFun

quadxapfun :: MixerFun

quadxapfun sfun demands = quadfun sfun demands rspins pspins yspins where

    rspins = Spins (-1)   1   1  (-1)
    pspins = Spins (-1)   1 (-1)   1
    yspins = Spins   1    1 (-1) (-1)

quadxap = Mixer quadxapfun quadFun
