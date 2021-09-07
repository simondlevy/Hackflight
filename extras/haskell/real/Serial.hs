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

    parserState = updateParser (SerialGuard serialAvailable serialByteIn) parserState'

    parserState' = [0] ++ parserState

updateParser :: SerialGuard -> ParserState -> ParserState

updateParser serialGuard  parserState = parserState'

  where

    parserState' = if (available serialGuard) then parserIdle else parserIdle

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
