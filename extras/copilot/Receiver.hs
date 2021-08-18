{--
  Haskell Copilot support for Hackflight

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}
{-# LANGUAGE DataKinds        #-}

module Receiver where

import Language.Copilot
import Copilot.Compile.C99

receiverThrottle :: Stream Double
receiverThrottle  = extern "receiverThrottle" Nothing

receiverRoll :: Stream Double
receiverRoll  = extern "receiverRoll" Nothing

receiverPitch :: Stream Double
receiverPitch  = extern "receiverPitch" Nothing

receiverYaw :: Stream Double
receiverYaw  = extern "receiverYaw" Nothing
