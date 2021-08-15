module Main where

import Language.Copilot
import Copilot.Compile.C99

import Prelude hiding ((>), (<), div)

receiverThrottle :: Stream Double
receiverThrottle  = extern "receiverThrottle" Nothing

spec = do

  trigger "runMotor" true [arg receiverThrottle]

-- Compile the spec
main = reify spec >>= compile "hackflight"
