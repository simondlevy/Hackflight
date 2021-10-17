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

addMotors :: Motors -> Motors -> Motors

addMotors (QuadMotors m11 m12 m13 m14) (QuadMotors m21 m22 m23 m24) 
  = QuadMotors (m11 + m21) (m12 + m22) (m13 + m23) (m14 + m24)

type Mixer = Demands -> Motors

-- addMotors :: Motors -> Motors -> Motors

quadXMWMixer :: Mixer

quadXMWMixer demands =

  --                 Th  RR  PF  YR
  QuadMotors (t - r + p - y)
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
  QuadMotors (t - r - p + y)
             (t + r + p + y)
             (t + r - p - y)
             (t - r + p - y)
  where 

    t = ((throttle demands) + 1) / 2 -- Map throttle from [-1,+1] to [0,1]
    r = roll demands
    p = pitch demands
    y = yaw demands
