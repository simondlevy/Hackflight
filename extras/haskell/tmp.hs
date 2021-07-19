module Tmp

where

data S = A Double Double

type F = Double -> Double -> S -> (Double, S)

loop :: (F, S) -> (F, S)

loop pc  =

      let t = 0

          f = fst pc

          (d, n) = f t d (snd pc)

      in loop (f, n)
