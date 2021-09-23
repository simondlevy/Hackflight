module Main where

import Language.Copilot
import Copilot.Compile.C99

import Prelude hiding ((>), (<), div)

spec = do

  trigger "tick"  true []

-- Compile the spec
main = reify spec >>= compile "timer"
