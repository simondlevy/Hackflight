module Main where

import Language.Copilot
import Copilot.Compile.C99

import Prelude hiding ((>), (<), div)

-- External temperature as a byte, range of -50C to 100C
temp :: Stream Float
temp = extern "temperature" Nothing

ctemp :: Stream Float
ctemp = temp

spec = do
  -- Triggers that fire when the ctemp is too low or too high,
  -- pass the current ctemp as an argument.
  trigger "heaton"  (ctemp < 18.0) [arg ctemp]
  trigger "heatoff" (ctemp > 21.0) [arg ctemp]

-- Compile the spec
main = reify spec >>= compile "hackflight"
