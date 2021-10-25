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

serialByteIn :: Stream Word8
serialByteIn = extern "stream_serialByte" Nothing
