{--
  Haskell Copilot support for Hackflight

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}
{-# LANGUAGE DataKinds        #-}

module Main where

import Language.Copilot
import Copilot.Compile.C99

receiverDemands :: Stream (Array 4 Double)
receiverDemands  = extern "receiverDemands" Nothing

motorValues :: Stream (Array 4 Double)
motorValues = [array [0, 0, 0, 0]] ++ motorValues

data MyDemands = MyDemands { throttle :: Stream Double }

spec = do

  let initialDemands = MyDemands (receiverDemands .!!0)

  let newMotorValues = [array [0.1, 0.2, 0.3, 0.4]] ++ motorValues

  trigger "runMotors" true [arg newMotorValues]

-- Compile the spec
main = reify spec >>= compile "hackflight"
