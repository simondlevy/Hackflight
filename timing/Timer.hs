module Main where

import Language.Copilot
import Copilot.Compile.C99

import Prelude hiding ((>), (<), div)

time_sec :: Stream Float
time_sec  = extern "copilot_time_sec" Nothing

spec = do

  trigger "tick"  true []

-- Compile the spec
main = reify spec >>= compile "timer"

