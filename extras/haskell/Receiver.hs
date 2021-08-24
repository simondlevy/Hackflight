{--
  Haskell Copilot support for Hackflight

  Copyright(C) 2021 on D.Levy

  MIT License
--}


module Receiver where

import Language.Copilot
import Copilot.Compile.C99

receiverThrottle :: Stream Double
receiverThrottle  = extern "copilot_receiverThrottle" Nothing

receiverRoll :: Stream Double
receiverRoll  = extern "copilot_receiverRoll" Nothing

receiverPitch :: Stream Double
receiverPitch  = extern "copilot_receiverPitch" Nothing

receiverYaw :: Stream Double
receiverYaw  = extern "copilot_receiverYaw" Nothing
