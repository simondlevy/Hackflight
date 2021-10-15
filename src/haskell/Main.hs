{--
  Haskell Copilot support for Hackflight

  Copyright(C) 2021 on D.Levy

  MIT License
--}

module Main where

import Language.Copilot
import Copilot.Compile.C99
import Prelude hiding((<), (>))

-- Core
import Demands
import Receiver

------------------------------------------------------------

receiver = makeReceiverWithTrim (AxisTrim 0.0 0.05 0.045) 4.0

spec = do

  let demands = getDemands receiver

  -- Send the motor values using the external C function
  trigger "stream_runHackflight" true [  arg $ throttle demands
                                       , arg $ roll demands
                                       , arg $ pitch demands
                                       , arg $ yaw demands 
                                       , arg $ receiverAux1 > 0
                                       , arg $ (throttle demands) < (-0.995)
                                      ]
-- Compile the spec
main = reify spec >>= compile "copilot"
