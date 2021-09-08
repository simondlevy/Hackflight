{--
  Haskell Copilot tests

  Copyright(C) 2021 on D.Levy

  MIT License
--}

module Test where

import Language.Copilot hiding(xor)
import Copilot.Compile.C99

import Utils

spec = do

  trigger "display" true [arg $ xor 3 4]

-- Compile the spec
main = reify spec >>= compile "test"
