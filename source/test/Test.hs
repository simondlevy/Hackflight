{--
  Haskell Copilot tests

  Copyright(C) 2021 on D.Levy

  MIT License
--}

module Test where

import Language.Copilot hiding(xor)
import Copilot.Library.Stacks
import Copilot.Compile.C99

import Utils

spec = do

  let s = stack 100 (0 :: Stream Float)

  trigger "display" true [arg $ xor 37 128]

-- Compile the spec
main = reify spec >>= compile "test"
