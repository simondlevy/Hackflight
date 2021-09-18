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

emptySerialBuffer :: SerialBuffer
emptySerialBuffer = SerialBuffer 0 0 0 0 0 0 0 0 0 0 0 0 0 0

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

-- parse mixer vehicleState = if serialAvailable then (SerialBuffer 0 0 0 0 0 0 0 0 0 0 0 0 0 0) else emptySerialBuffer 
parse mixer vehicleState = emptySerialBuffer 

  where 

    c = serialByteIn

    size = if pState' == pGotArrow then c else size'
    size' = [0] ++ size

    index = if pState' == pInPayload then index' + 1 else  0
    index' = [0] ++ index

    -- Parser state transition function
    pState = if pState' == pIdle && c == 24 then pGotStart
                  else if pState' == pGotStart && c == 77 then pGotM
                  else if pState' == pGotM && (c == 60 || c == 62) then pGotArrow
                  else if pState' == pGotArrow then pGotSize
                  else if pState' == pGotSize then pInPayload
                  else if pState' == pInPayload && index < size then pInPayload
                  else if pState' == pInPayload then pIdle
                  else pState'
    pState' = [0] ++ pState

----------------------------------------------------------

serialAvailable :: Stream Bool
serialAvailable = extern "copilot_serialAvailable" Nothing

serialByteIn :: Stream Word8
serialByteIn = extern "copilot_serialByte" Nothing
