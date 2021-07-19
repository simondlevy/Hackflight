module Tmp

where

import Demands

data PidState = AltHoldState Double Double

type PidFun = Double -> Demands -> PidState -> (Demands, PidState)

loop :: (PidFun, PidState) -> (PidFun, PidState)

loop pidController  =

      let t = 0

          controllerFun = fst pidController

          (d, n) = controllerFun t d (snd pidController)

      in loop (controllerFun, n)
