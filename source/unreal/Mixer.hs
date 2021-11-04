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

data Motors = QuadMotors { m1 :: SFloat
                         , m2 :: SFloat
                         , m3 :: SFloat
                         , m4 :: SFloat }

type Mixer = Demands -> Motors

quadXAPMixer demands =

  --                 Th  RR  PF  YR
  QuadMotors (constrain $ t - r - p + y)
             (constrain $ t + r + p + y)
             (constrain $ t + r - p - y)
             (constrain $ t - r + p - y)
  where 

    t = ((throttle demands) + 1) / 2 -- Map throttle from [-1,+1] to [0,1]
    r = roll demands
    p = pitch demands
    y = yaw demands
