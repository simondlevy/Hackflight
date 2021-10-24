{--
  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module Main where

import Language.Copilot
import Copilot.Compile.C99
import Prelude hiding((<), (>), (++), not)

spec = do

  trigger "report" true []

-- Compile the spec
main = reify spec >>= compile "copilot"
