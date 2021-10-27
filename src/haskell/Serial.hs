{--
  Hackflight serial comms support

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}
{-# LANGUAGE DataKinds        #-}

module Serial where

import Language.Copilot
import Prelude hiding((++), (==), take, drop)

import Utils


serialAvailable :: Stream Bool
serialAvailable = extern "stream_serialAvailable" Nothing

serialByte :: Stream Word8
serialByte = extern "stream_serialByte" Nothing
