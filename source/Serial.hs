{--
  Hackflight serial comms

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module Serial where

import Language.Copilot hiding(xor)
import Copilot.Language.Operators.BitWise((.|.), (.<<.))

import Receiver
import State
import Mixer
import Utils(xor)


data InputWords = InputWords {  w00 :: Stream Word32
                              , w01 :: Stream Word32 
                              , w02 :: Stream Word32 
                              , w03 :: Stream Word32 
                             }

data OutputValues = OutputValues {  v00 :: Stream Float
                                  , v01 :: Stream Float
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
                                  }
 
data SerialBuffer = SerialBuffer {  inputReady :: Stream Bool 
                                  , count      :: Stream Word8
                                  , msgtype    :: Stream Word8
                                  , inwords    :: InputWords
                                  , outvals    :: OutputValues 
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

-- Helper fucntions

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

getOutputSize :: Stream Word8 -> Stream Word8
getOutputSize msgtype = if msgtype == 121 then 6
                 else if msgtype == 122 then 12
                 else 0

getOutputValue :: Stream Bool -> State -> Stream Word8 -> Stream Word8
  -> Stream Float
getOutputValue ready vehicleState msgtype index = 
  if not ready then 0
  else if msgtype == 121 then receiverValue index
  else if msgtype == 122 then vehicleStateValue vehicleState index
  else 0


bufferInput ::   Stream Bool   -- inPayload flag
              -> Stream Word8  -- bufferIndex
              -> Stream Word8  -- payloadIndex
              -> Stream Word8  -- byte
              -> Stream Word32 -- word
              -> Stream Word32 -- new word

bufferInput inPayload bufferIndex payloadIndex byte word
  = if inPayload && (div payloadIndex 4) == bufferIndex
    -- then word .|. ((cast byte) .<<. (mod bufferIndex 4))
    then (cast byte)
    else word


-- Parser function

parse :: Mixer -> State -> (SerialBuffer, Motors)

parse mixer vehicleState = (serialBuffer, motors)

  where 

    c = if serialAvailable then serialByteIn else 0

    size = if pstate' == pGotArrow then c else size'
    size' = [0] ++ size

    index = if pstate' == pInPayload then index' + 1 else  0
    index' = [0] ++ index

    inPayload = msgtype' >= 200 && pstate == pInPayload

    msgtype = if pstate' == pGotSize then c else msgtype'
    msgtype' = [0] ++ msgtype

    crc = if pstate' == pGotArrow then c
          else if pstate' == pInPayload then xor crc' c
          else 0
    crc' = [0] ++ crc
 
    pstate = if pstate' == pIdle && c == 36 then pGotStart
                  else if pstate' == pGotStart && c == 77 then pGotM
                  else if pstate' == pGotM && (c == 60 || c == 62) then pGotArrow
                  else if pstate' == pGotArrow then pGotSize
                  else if pstate' == pGotSize then pInPayload
                  else if pstate' == pInPayload && index < size then pInPayload
                  else if pstate' == pInPayload then pIdle
                  else pstate'

    inputReady = pstate' == pInPayload && pstate == pIdle

    pstate' = [0] ++ pstate

    w00 = bufferInput inPayload 0 index c w00' 
    w00' = [0] ++ w00
    w01 = 0 -- bufferInput inPayload 1 index c w01' 
    w01' = [0] ++ w01
    w02 = 0 -- bufferInput inPayload 2 index c w02' 
    w02' = [0] ++ w02
    w03 = 0 -- bufferInput inPayload 3 index c w03' 
    w03' = [0] ++ w03

    ready = pstate == pIdle && crc == c

    count = if ready then getOutputSize msgtype else 0

    motorsReady = ready && msgtype == 215

    motor1 = if motorsReady then input1 else motor1'
    motor2 = if motorsReady then input2 else motor1'
    motor3 = if motorsReady then input3 else motor1'
    motor4 = if motorsReady then input4 else motor1'

    motor1' = [0] ++ motor1
    motor2' = [0] ++ motor2
    motor3' = [0] ++ motor3
    motor4' = [0] ++ motor4

    v00 = getOutputValue ready vehicleState msgtype 0
    v01 = getOutputValue ready vehicleState msgtype 1
    v02 = getOutputValue ready vehicleState msgtype 2
    v03 = getOutputValue ready vehicleState msgtype 3
    v04 = getOutputValue ready vehicleState msgtype 4
    v05 = getOutputValue ready vehicleState msgtype 5
    v06 = getOutputValue ready vehicleState msgtype 6
    v07 = getOutputValue ready vehicleState msgtype 7
    v08 = getOutputValue ready vehicleState msgtype 8
    v09 = getOutputValue ready vehicleState msgtype 9
    v10 = getOutputValue ready vehicleState msgtype 10
    v11 = getOutputValue ready vehicleState msgtype 11

    outputValues = OutputValues v00 v01 v02 v03 v04 v05 v06 v07 v08 v09 v10 v11

    inputBuffer = InputWords w00 w01 w02 w03

    serialBuffer = SerialBuffer inputReady count msgtype inputBuffer outputValues

    motors = QuadMotors motor1 motor2 motor3 motor4

----------------------------------------------------------

serialAvailable :: Stream Bool
serialAvailable = extern "copilot_serialAvailable" Nothing

serialByteIn :: Stream Word8
serialByteIn = extern "copilot_serialByte" Nothing

input1 :: Stream Float
input1 = extern "copilot_input1" Nothing

input2 :: Stream Float
input2 = extern "copilot_input2" Nothing

input3 :: Stream Float
input3 = extern "copilot_input3" Nothing

input4 :: Stream Float
input4 = extern "copilot_input4" Nothing
