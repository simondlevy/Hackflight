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


type SerialBuffer = [SWord8]

buff :: SerialBuffer

buff = [   0::SWord8
         , 0::SWord8
       ]

insert :: SerialBuffer -> SWord8 -> SWord8 -> SerialBuffer
insert buff index value = [0, 0]

serialAvailable :: Stream Bool
serialAvailable = extern "stream_serialAvailable" Nothing

serialByteIn :: Stream Word8
serialByteIn = extern "stream_serialByte" Nothing
