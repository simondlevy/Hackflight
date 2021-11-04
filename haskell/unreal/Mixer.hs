{--
  Mixer type

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module Mixer where

import Language.Copilot

import Demands
import Utils

data Motors = Motors { m1 :: SFloat
                     , m2 :: SFloat
                     , m3 :: SFloat
                     , m4 :: SFloat }

type Mixer = Demands -> Motors

quadxap :: Mixer

quadxap demands = Motors m1 m2 m3 m4 where

  t = ((throttle demands) + 1) / 2 -- Map throttle from [-1,+1] to [0,1]

  r = roll demands
  p = pitch demands
  y = yaw demands

  m1 = constrain $ t - r - p + y
  m2 = constrain $ t + r + p + y
  m3 = constrain $ t + r - p - y
  m4 = constrain $ t - r + p - y

