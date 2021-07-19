{--
  Socket-based multicopter control

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module Tmp

where

import State
import Demands

data PidState = AltHoldState Double Double

type PidFun = Time -> Demands -> PidState -> (Demands, PidState)

loop :: (PidFun, PidState) -> (PidFun, PidState)

loop pidController  =

      let t = 0

          controllerFun = fst pidController

          (d, n) = controllerFun t d (snd pidController)

      in loop (controllerFun, n)
