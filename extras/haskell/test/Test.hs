{--
  Haskell Copilot tests

  Copyright(C) 2021 on D.Levy

  MIT License
--}

module Test where

import Language.Copilot
import Copilot.Compile.C99

import Utils

spec = do

  let x = deg2rad 0  

  trigger "display" true [arg x]

-- Compile the spec
main = reify spec >>= compile "test"
