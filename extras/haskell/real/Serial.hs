{--
  Hackflight serial comms

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module Serial where

import Language.Copilot

import State
import Demands

data SerialGuard = SerialGuard { available :: Stream Bool, value :: Stream Word8 }

type ParserState = Stream Word8

getSerialOut :: State -> Demands -> SerialGuard

getSerialOut _vehicleState _demands = SerialGuard false 0

  where 

    dataSize = 

      if not serialAvailable then dataSize'
      else if parserState == parserHeaderArrow then serialByteIn
      else dataSize'

    direction =

      if not serialAvailable then direction'
      else if parserState == parserHeaderM && serialByteIn == 0x3E then 0 -- '>'
      else if parserState == parserHeaderM && serialByteIn == 0x3C then 1 -- '<'
      else direction'

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

      else if parserState == parserHeaderCmd
          then parserIdle

      else parserIdle
         

    -- Parser state variables
    parserState' = [0] ++ parserState :: Stream Word8
    direction' = [0] ++ direction :: Stream Word8
    dataSize' = [0] ++ dataSize :: Stream Word8

    -- Parser constants
    parserIdle        = 0
    parserHeaderStart = 1
    parserHeaderM     = 2
    parserHeaderArrow = 3
    parserHeaderSize  = 4
    parserHeaderCmd   = 5




----------------------------------------------------------

serialAvailable :: Stream Bool
serialAvailable = extern "copilot_serialAvailable" Nothing

serialByteIn :: Stream Word8
serialByteIn = extern "copilot_serialByte" Nothing
