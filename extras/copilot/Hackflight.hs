module Main where

import Language.Copilot
import Copilot.Compile.C99

import Prelude hiding ((>), (<), div)

-- External temperature as a byte, range of -50C to 100C
throttle :: Stream Double
throttle = extern "throttle" Nothing

spec = do
  trigger "runMotor" true [arg throttle]

-- Compile the spec
main = reify spec >>= compile "hackflight"
