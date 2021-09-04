{--
  Mixer type

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

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

quadXAPMixer _ demands =

  QuadMotors  (constrain (t - r - p + y))
              (constrain (t + r + p + y))
              (constrain (t + r - p - y))
              (constrain (t - r + p - y))

  where 

    t = throttle demands
    r = roll demands
    p = pitch demands
    y = yaw demands
