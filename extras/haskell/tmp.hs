module Tmp

where

data S = A Double Double

type F = Double -> Double -> S -> (Double, S)

loop :: (F, S) -> (F, S)

loop p  =

      let t = 0

          f = fst p

          (d, n) = f t d (snd p)

      in loop (f, n)
