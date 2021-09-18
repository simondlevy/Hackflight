{--
  Hackflight serial comms

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module Serial where

import Language.Copilot hiding(xor)

import Receiver
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

receiverValue :: Stream Word8 -> Stream Float
receiverValue index =
  if index == 0 then receiverThrottle
  else if index == 1 then receiverRoll
  else if index == 2 then receiverPitch
  else if index == 3 then receiverYaw
  else if index == 4 then receiverAux1
  else 0

vehicleStateValue :: State -> Stream Word8 -> Stream Float
vehicleStateValue vehicleState index =
  if index == 0 then x vehicleState
  else if index == 1 then dx vehicleState
  else if index == 2 then y vehicleState
  else if index == 3 then dy vehicleState
  else if index == 4 then z vehicleState
  else if index == 5 then dz vehicleState
  else if index == 6 then phi vehicleState
  else if index == 7 then dphi vehicleState
  else if index == 8 then theta vehicleState
  else if index == 9 then dtheta vehicleState
  else if index == 10 then psi vehicleState
  else if index == 11 then dpsi vehicleState
  else 0

getOutputValue :: Stream Bool -> State -> Stream Word8 -> Stream Word8 -> Stream Float

getOutputValue ready vehicleState mtype index = 
  if not ready then 0
  else if mtype == 121 then receiverValue index
  else if mtype == 122 then vehicleStateValue vehicleState index
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

    ready = pstate == pIdle && crc == c

    count = if inPayload then -1
            else if pstate == pIdle && crc == c then mtype2count mtype
            else 0

    input = if inPayload then c else 0

    v01 = getOutputValue ready vehicleState mtype 0
    v02 = 0
    v03 = 0
    v04 = 0
    v05 = 0
    v06 = 0
    v07 = 0
    v08 = 0
    v09 = 0
    v10 = 0
    v11 = 0
    v12 = 0

    output = OutputValues v01 v02 v03 v04 v05 v06 v07 v08 v09 v10 v11 v12

----------------------------------------------------------

serialAvailable :: Stream Bool
serialAvailable = extern "copilot_serialAvailable" Nothing

serialByteIn :: Stream Word8
serialByteIn = extern "copilot_serialByte" Nothing
