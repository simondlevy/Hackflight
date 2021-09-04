{--
  Mixer type

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module Mixer where

import Language.Copilot

import Demands
import Utils(constrain)

data Motors = QuadMotors { m1 :: Stream Float
                         , m2 :: Stream Float
                         , m3 :: Stream Float
                         , m4 :: Stream Float }

type Mixer = Stream Bool -> Demands -> Motors

quadXAPMixer :: Mixer

quadXAPMixer  failsafe demands =

  QuadMotors  (check $ t - r - p + y)
              (check $ t + r + p + y)
              (check $ t + r - p - y)
              (check $ t - r + p - y)
  where 

    check x = if failsafe then 0 else constrain x

    t = throttle demands
    r = roll demands
    p = pitch demands
    y = yaw demands
