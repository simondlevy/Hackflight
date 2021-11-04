{--
  Mixer type

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module Mixer where

import Language.Copilot

import Demands
import Safety
import Utils

data Motors = QuadMotors { m1 :: SFloat
                         , m2 :: SFloat
                         , m3 :: SFloat
                         , m4 :: SFloat }

type Mixer = Safety -> Demands -> Motors

quadXMWMixer :: Mixer

quadXMWMixer safety demands =

  --                 Th  RR  PF  YR
  QuadMotors (check $ t - r + p - y)
             (check $ t - r - p + y)
             (check $ t + r + p + y)
             (check $ t + r - p - y)
  where 

    t = ((throttle demands) + 1) / 2 -- Map throttle from [-1,+1] to [0,1]
    r = roll demands
    p = pitch demands
    y = yaw demands

    check x = if (armed safety) && (not $ failsafe safety) && (t > 0)
              then constrain x
              else 0

quadXAPMixer :: Mixer

quadXAPMixer safety demands =

  --                 Th  RR  PF  YR
  QuadMotors (check $ t - r - p + y)
             (check $ t + r + p + y)
             (check $ t + r - p - y)
             (check $ t - r + p - y)
  where 

    t = ((throttle demands) + 1) / 2 -- Map throttle from [-1,+1] to [0,1]
    r = roll demands
    p = pitch demands
    y = yaw demands

    check x = if (armed safety) && (not $ failsafe safety) && (t > 0)
              then constrain x
              else 0

