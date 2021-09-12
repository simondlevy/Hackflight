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
import Utils(constrain)

data Motors = QuadMotors { m1 :: Stream Float
                         , m2 :: Stream Float
                         , m3 :: Stream Float
                         , m4 :: Stream Float }

type Mixer = Safety -> Demands -> Motors

quadXAPMixer :: Mixer

quadXAPMixer safety demands =

  QuadMotors  (check $ t - r - p + y)
              (check $ t + r + p + y)
              (check $ t + r - p - y)
              (check $ t - r + p - y)
  where 

    check x = if ((not (armed safety)) || (failsafe safety)) then 0 else constrain x

    t = throttle demands
    r = roll demands
    p = pitch demands
    y = yaw demands
