{--
  Hackflight serial comms support

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}
{-# LANGUAGE DataKinds        #-}

module Serial where

import Language.Copilot

stream_serialAvailable :: Stream Bool
stream_serialAvailable = extern "stream_serialAvailable" Nothing

stream_serialByte :: Stream Word8
stream_serialByte = extern "stream_serialByte" Nothing
