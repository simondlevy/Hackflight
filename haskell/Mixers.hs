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

dmds :: Demands -> (SFloat, SFloat, SFloat, SFloat)

-- Map throttle demand from [-1,+1] to [0,1]
dmds demands = (((throttle demands) + 1) / 2, roll demands, pitch demands, yaw demands )

mix :: SafetyFun -> Demands -> Mixer -> Motors

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

