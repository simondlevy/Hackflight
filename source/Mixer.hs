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

type Mixer = SBool -> Demands -> Motors

quadxmw :: Mixer

quadxmw zeroed demands = Motors m1 m2 m3 m4 where

  -- Map throttle demand from [-1,+1] to [0,1]
  t = ((throttle demands) + 1) / 2
  
  r = roll demands
  p = pitch demands
  y = yaw demands

  m1 = saferun $ t - r + p  - y
  m2 = saferun $ t - r - p  + y
  m3 = saferun $ t + r + p  + y
  m4 = saferun $ t + r - p  - y

  saferun m = if zeroed then 0 else m

quadxap :: Mixer

quadxap zeroed demands = Motors m1 m2 m3 m4 where

  -- Map throttle demand from [-1,+1] to [0,1]
  t = ((throttle demands) + 1) / 2
  
  r = roll demands
  p = pitch demands
  y = yaw demands

  m1 = saferun $ t - r - p  + y
  m2 = saferun $ t + r + p  + y
  m3 = saferun $ t + r - p  - y
  m4 = saferun $ t - r + p  - y

  saferun m = if zeroed then 0 else m
