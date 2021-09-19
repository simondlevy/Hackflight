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

addMotors :: Motors -> Motors -> Motors

addMotors (QuadMotors m11 m12 m13 m14) (QuadMotors m21 m22 m23 m24) 
  = QuadMotors (m11 + m21) (m12 + m22) (m13 + m23) (m14 + m24)

type Mixer = Safety -> Demands -> Motors

-- addMotors :: Motors -> Motors -> Motors

quadXAPMixer :: Mixer

quadXAPMixer safety demands =

  QuadMotors (check $ t - r - p + y)
             (check $ t + r + p + y)
             (check $ t + r - p - y)
             (check $ t - r + p - y)
  where 

    check x = if ((not (armed safety)) || (failsafe safety)) then 0 else constrain x

    t = throttle demands
    r = roll demands
    p = pitch demands
    y = yaw demands
