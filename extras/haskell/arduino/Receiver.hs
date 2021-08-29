{--
  Haskell Copilot support for RC receivers

  Copyright(C) 2021 on D.Levy

  MIT License
--}


module Receiver where

import Language.Copilot
import Copilot.Compile.C99

-- Externals --------------------------------------------------

receiverThrottle :: Stream Float
receiverThrottle  = extern "copilot_receiverThrottle" Nothing

receiverRoll :: Stream Float
receiverRoll  = extern "copilot_receiverRoll" Nothing

receiverPitch :: Stream Float
receiverPitch  = extern "copilot_receiverPitch" Nothing

receiverYaw :: Stream Float
receiverYaw  = extern "copilot_receiverYaw" Nothing

receiverLostSignal :: Stream Bool
receiverLostSignal  = extern "copilot_receiverLostSignal" Nothing

-- Internals -------------------------------------------------

data Receiver = Receiver {  throttleMargin :: Stream Float
                          , throttleExpo :: Stream Float
                          , cyclicExpo :: Stream Float
                          , cyclicRate :: Stream Float
                          , auxTheshold :: Stream Float

                         } deriving (Show)


