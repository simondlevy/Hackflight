{-# LANGUAGE RebindableSyntax #-}
{-# LANGUAGE DataKinds        #-}

module Main where

import Language.Copilot
import Copilot.Compile.C99


receiverDemands :: Stream (Array 4 Double)
receiverDemands  = extern "receiverDemands" Nothing

motorValues :: Stream (Array 4 Double)
motorValues = [array [0,0,0,0]] ++ motorValues

spec = do

  trigger "runMotors" true [arg motorValues]

-- Compile the spec
main = reify spec >>= compile "hackflight"
