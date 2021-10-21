{--
  Hackflight serial comms support

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}
{-# LANGUAGE DataKinds        #-}

module Serial where

import Language.Copilot

import Utils

data SerialBuffer = SerialBuffer { byte00 :: SWord8
                                 }

buffer :: Stream (Array 128 Word8)
buffer = extern "stream_buffer" Nothing

serialAvailable :: Stream Bool
serialAvailable = extern "stream_serialAvailable" Nothing

serialByteIn :: Stream Word8
serialByteIn = extern "stream_serialByte" Nothing
