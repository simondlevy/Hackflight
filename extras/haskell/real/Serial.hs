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

    parserState = if not serialAvailable then parserState'

                  else if parserState == parserIdle && serialByteIn == 0x24 -- '$'
                      then parserHeaderStart

                  else if parserState == parserHeaderStart && serialByteIn == 0x4D -- 'M'
                      then parserHeaderM

                  else parserIdle
                     

    parserState' = [0] ++ parserState :: Stream Word8

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
