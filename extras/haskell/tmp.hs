module Tmp

where

import Demands

data S = AltHoldState Double Double

type F = Double -> Demands -> S -> (Demands, S)

loop :: (F, S) -> (F, S)

loop pc  =

      let t = 0

          controllerFun = fst pc

          (d, n) = controllerFun t d (snd pc)

      in loop (controllerFun, n)
