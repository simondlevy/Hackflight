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

data OutputValues = OutputValues {  v01 :: Stream Float
                                  , v02 :: Stream Float
                                  , v03 :: Stream Float
                                  , v04 :: Stream Float
                                  , v05 :: Stream Float
                                  , v06 :: Stream Float
                                  , v07 :: Stream Float
                                  , v08 :: Stream Float
                                  , v09 :: Stream Float
                                  , v10 :: Stream Float
                                  , v11 :: Stream Float
                                  , v12 :: Stream Float
                                  }
 
data SerialBuffer = SerialBuffer {  count   :: Stream Word8
                                  , mtype   :: Stream Word8
                                  , input   :: Stream Word8
                                  , outvals :: OutputValues 
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

-- Helper fucntion

mtype2count :: Stream Word8 -> Stream Word8
mtype2count mt = if mt == 121 then 6
                 else if mt == 122 then 12
                 else if mt == 123 then 1
                 else 0

-- Parser function

parse :: Mixer -> State -> SerialBuffer

parse mixer vehicleState = SerialBuffer count mtype input output

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

    count = if inPayload then -1
            else if pstate == pIdle && crc == c then mtype2count mtype
            else 0

    input = 0

    output = OutputValues 0 0 0 0 0 0 0 0 0 0 0 0

----------------------------------------------------------

serialAvailable :: Stream Bool
serialAvailable = extern "copilot_serialAvailable" Nothing

serialByteIn :: Stream Word8
serialByteIn = extern "copilot_serialByte" Nothing
