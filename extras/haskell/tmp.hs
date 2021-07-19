module Tmp

where

data S = A Double Double

type F = Double -> Double -> S -> (Double, S)

loop :: (F, S) -> (F, S)

loop pc  =

      let t = 0

          controllerFun = fst pc

          (d, n) = controllerFun t d (snd pc)

      in loop (controllerFun, n)
