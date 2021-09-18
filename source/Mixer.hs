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

data Motor = Motor {  index :: Stream Word8
                    , value :: Stream Float }

data Motors = QuadMotors { m1 :: Motor
                         , m2 :: Motor
                         , m3 :: Motor
                         , m4 :: Motor }

type Mixer = Safety -> Demands -> Motors

quadXAPMixer :: Mixer

quadXAPMixer safety demands =

  QuadMotors  (Motor 0 (check $ t - r - p + y))
              (Motor 1 (check $ t + r + p + y))
              (Motor 2 (check $ t + r - p - y))
              (Motor 3 (check $ t - r + p - y))
  where 

    check x = if ((not (armed safety)) || (failsafe safety)) then 0 else constrain x

    t = throttle demands
    r = roll demands
    p = pitch demands
    y = yaw demands
