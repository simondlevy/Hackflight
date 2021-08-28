{--
  Haskell Copilot support for Hackflight

  Copyright(C) 2021 on D.Levy

  MIT License
--}


module Receiver where

import Language.Copilot
import Copilot.Compile.C99

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

receiverReady :: Stream Bool
receiverReady  = extern "copilot_receiverReady" Nothing

receiverInArmedState :: Stream Bool
receiverInArmedState  = extern "copilot_receiverInArmedState" Nothing

receiverInactive :: Stream Bool
receiverInactive  = extern "copilot_receiverInactive" Nothing
