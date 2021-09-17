{--
  Hackflight serial comms

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module Serial where

import Language.Copilot hiding(xor)

import State
import Mixer
import Utils(xor)


data SerialBuffer = SerialBuffer {  count    :: Stream Word8
                                  , cmdid    :: Stream Word8
                                  , output01 :: Stream Float
                                  , output02 :: Stream Float
                                  , output03 :: Stream Float
                                  , output04 :: Stream Float
                                  , output05 :: Stream Float
                                  , output06 :: Stream Float
                                  , output07 :: Stream Float
                                  , output08 :: Stream Float
                                  , output09 :: Stream Float
                                  , output10 :: Stream Float
                                  , output11 :: Stream Float
                                  , output12 :: Stream Float
                                  }
parse :: Mixer -> State -> SerialBuffer

parse mixer vehicleState = SerialBuffer 0 0 0 0 0 0 0 0 0 0 0 0 0 0

  where 

    -- Parser state constants
    parserIdle        = 0
    parserHeaderStart = 1
    parserHeaderM     = 2
    parserHeaderArrow = 3
    parserHeaderSize  = 4
    parserHeaderCmd   = 5

{--
    command = 

      if not serialAvailable then command'
      else if parserState == parserHeaderSize then serialByteIn
      else command'

    checksum = 

      if not serialAvailable then checksum'
      else if parserState == parserHeaderArrow then serialByteIn
      else if parserState == parserHeaderSize then xor checksum' serialByteIn
      else checksum'

    offset = 

      if not serialAvailable then offset'
      else if parserState == parserHeaderArrow then 0
      else if parserState == parserHeaderCmd && offset' < dataSize then offset' + 1
      else offset'

    dataSize = 

      if not serialAvailable then dataSize'
      else if parserState == parserHeaderArrow then serialByteIn
      else dataSize'

    parserState =

      if not serialAvailable then parserState'

      else if parserState == parserIdle && serialByteIn == 0x24 -- '$'
          then parserHeaderStart

      else if parserState == parserHeaderStart && serialByteIn == 0x4D -- 'M'
          then parserHeaderM

      else if parserState == parserHeaderM && serialByteIn == 0x3E -- '>'
          then parserHeaderArrow

      else if parserState == parserHeaderM && serialByteIn == 0x3C -- '<'
          then parserHeaderArrow

      else if parserState == parserHeaderArrow
          then parserHeaderSize

      else if parserState == parserHeaderSize
          then parserHeaderCmd

      else if parserState == parserHeaderCmd && offset < dataSize
          then parserHeaderCmd

      else if parserState == parserHeaderCmd
              && offset == dataSize
              && checksum == serialByteIn
          then parserHeaderCmd

      else parserIdle

    -- Parser state variables
    parserState' = [0] ++ parserState :: Stream Word8
    dataSize' = [0] ++ dataSize :: Stream Word8
    offset' = [0] ++ offset :: Stream Word8
    checksum' = [0] ++ checksum :: Stream Word8
    command' = [0] ++ command :: Stream Word8

--}

----------------------------------------------------------

serialAvailable :: Stream Bool
serialAvailable = extern "copilot_serialAvailable" Nothing

serialByteIn :: Stream Word8
serialByteIn = extern "copilot_serialByte" Nothing
