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

type Mixer = (SFloat -> SFloat) -> Demands -> Motors

quadxmw :: Mixer

quadxmw fun demands = Motors m1 m2 m3 m4 where

  -- Map throttle demand from [-1,+1] to [0,1]
  t = ((throttle demands) + 1) / 2
  
  r = roll demands
  p = pitch demands
  y = yaw demands

  m1 = fun $ t - r + p  - y
  m2 = fun $ t - r - p  + y
  m3 = fun $ t + r + p  + y
  m4 = fun $ t + r - p  - y

quadxap :: Mixer

quadxap fun demands = Motors m1 m2 m3 m4 where

  -- Map throttle demand from [-1,+1] to [0,1]
  t = ((throttle demands) + 1) / 2
  
  r = roll demands
  p = pitch demands
  y = yaw demands

  m1 = fun $ t - r - p  + y
  m2 = fun $ t + r + p  + y
  m3 = fun $ t + r - p  - y
  m4 = fun $ t - r + p  - y
