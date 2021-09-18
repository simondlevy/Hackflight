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


data SerialBuffer = SerialBuffer {  count  :: Stream Word8
                                  , mtype    :: Stream Word8
                                  , input    :: Stream Word8
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

-- Parser state constants

type ParserState = Stream Word8

pIdle :: ParserState
pIdle = 0

pGotStart :: ParserState
pGotStart = 1

pGotM :: ParserState
pGotM = 2

pGotArrow :: ParserState
pGotArrow = 3

pGotSize :: ParserState
pGotSize = 4

pInPayload :: ParserState
pInPayload = 5

parse :: Mixer -> State -> SerialBuffer

parse mixer vehicleState = SerialBuffer count
                                        mtype
                                        input
                                        0
                                        0
                                        0
                                        0
                                        0
                                        0
                                        0
                                        0
                                        0
                                        0
                                        0
                                        0

  where 

    c = serialByteIn

    size = if pstate' == pGotArrow then c else size'
    size' = [0] ++ size

    index = if pstate' == pInPayload then index' + 1 else  0
    index' = [0] ++ index

    incoming = mtype' >= 200
    inPayload = incoming && pstate == pInPayload

    mtype = if pstate' == pGotSize then c else mtype'
    mtype' = [0] ++ mtype

    crc = if pstate' == pGotArrow then c
          else if pstate' == pInPayload then xor crc' c
          else 0
    crc' = [0] ++ crc
 
    pstate = if pstate' == pIdle && c == 24 then pGotStart
                  else if pstate' == pGotStart && c == 77 then pGotM
                  else if pstate' == pGotM && (c == 60 || c == 62) then pGotArrow
                  else if pstate' == pGotArrow then pGotSize
                  else if pstate' == pGotSize then pInPayload
                  else if pstate' == pInPayload && index < size then pInPayload
                  else if pstate' == pInPayload then pIdle
                  else pstate'
    pstate' = [0] ++ pstate

    count = if inPayload then -1 else 0

    input = 0

----------------------------------------------------------

serialAvailable :: Stream Bool
serialAvailable = extern "copilot_serialAvailable" Nothing

serialByteIn :: Stream Word8
serialByteIn = extern "copilot_serialByte" Nothing
