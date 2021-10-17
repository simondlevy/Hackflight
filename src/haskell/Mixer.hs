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

data Spins = Spins { s1 :: SFloat
                   , s2 :: SFloat
                   , s3 :: SFloat
                   , s4 :: SFloat }

runMixer :: Demands -> Spins -> Motors

runMixer demands spins = Motors m1 m2 m3 m4 where

  -- Map throttle demand from [-1,+1] to [0,1]
  t = ((throttle demands) + 1) / 2

  m1 = (s1 spins)
  m2 = 0
  m3 = 0
  m4 = 0

type Mixer = Demands -> Motors

quadXMWMixer :: Mixer

quadXMWMixer demands =

  --                 Th  RR  PF  YR
  Motors (t - r + p - y)
             (t - r - p + y)
             (t + r + p + y)
             (t + r - p - y)
  where 

    t = ((throttle demands) + 1) / 2 -- Map throttle from [-1,+1] to [0,1]
    r = roll demands
    p = pitch demands
    y = yaw demands

quadXAPMixer :: Mixer

quadXAPMixer demands =

  --                 Th  RR  PF  YR
  Motors (t - r - p + y)
             (t + r + p + y)
             (t + r - p - y)
             (t - r + p - y)
  where 

    t = ((throttle demands) + 1) / 2 -- Map throttle from [-1,+1] to [0,1]
    r = roll demands
    p = pitch demands
    y = yaw demands
