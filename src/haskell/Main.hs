{--
  Haskell Copilot support for Hackflight

  Copyright(C) 2021 on D.Levy

  MIT License
--}

module Main where

import Language.Copilot
import Copilot.Compile.C99

spec = do

  trigger "copilot_writeMotors" true []

-- Compile the spec
main = reify spec >>= compile "copilot"
