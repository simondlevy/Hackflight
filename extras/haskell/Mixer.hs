{--
  Mixer type

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module Mixer where

import Language.Copilot

import Demands
import Utils(constrain)

data QuadXAPMixer = QuadXAPMixer

getMotors :: QuadXAPMixer -> Demands -> (Stream Float, Stream Float, Stream Float, Stream Float)

getMotors mixer demands = 
  (  constrain (t - r - p + y)
   , constrain (t + r + p + y)
   , constrain (t + r - p - y)
   , constrain (t - r + p - y) )

  where 

    t = throttle demands
    r = roll demands
    p = pitch demands
    y = yaw demands


