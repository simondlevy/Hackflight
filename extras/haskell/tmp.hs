module Tmp

where

data S = A Double Double

type F = Double -> S -> (Double, S)

loop :: (F, S) -> (F, S)

loop p  =

      let f = fst p

          (d, n) = f d (snd p)

      in loop (f, n)
