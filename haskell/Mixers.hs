{--
  Mixer types

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module Mixers where

import Language.Copilot

import Demands
import Utils

data Motors = QuadMotors { m1 :: SFloat
                         , m2 :: SFloat
                         , m3 :: SFloat
                         , m4 :: SFloat }

type Mixer = (SFloat -> SFloat) -> Demands -> Motors

dmds :: Demands -> (SFloat, SFloat, SFloat, SFloat)

-- Map throttle demand from [-1,+1] to [0,1]
dmds demands = (((throttle demands) + 1) / 2, roll demands, pitch demands, yaw demands )

quadxmw :: Mixer

quadxmw fun demands = QuadMotors m1 m2 m3 m4 where

  (t, r, p, y) = dmds demands

  m1 = fun $ t - r + p  - y
  m2 = fun $ t - r - p  + y
  m3 = fun $ t + r + p  + y
  m4 = fun $ t + r - p  - y

quadxap :: Mixer

quadxap fun demands = QuadMotors m1 m2 m3 m4 where

  (t, r, p, y) = dmds demands

  m1 = fun $ t - r - p  + y
  m2 = fun $ t + r + p  + y
  m3 = fun $ t + r - p  - y
  m4 = fun $ t - r + p  - y
