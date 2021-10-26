{--
  Hackflight serial parsing

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}
{-# LANGUAGE DataKinds        #-}

module Parser where

import Language.Copilot hiding(xor)
import Prelude hiding((==), (>), (>=), (<=), (&&), (||), (++), (+), not)

import Utils

data ParserState = ParserState { sending :: SBool }

parse :: SWord8 -> ParserState

parse byte = ParserState sending where

  -- Checksum transition function
  crc = if state == 4 then byte
      else if state == 5  then crc'
      else 0

  -- Parser state transition function
  state  = if state' == 0 && byte == 36 then 1
      else if state' == 1 && byte == 77 then 2
      else if state' == 2 && (byte == 60 || byte == 62) then 3
      else if state' == 3 then 4
      else if state' == 4 then 5
      else if state' == 5 then 0
      else state'

  sending = state == 5 && crc == byte;

  -- State variables
  state'     = [0] ++ state :: SWord8
  crc'       = [0] ++ crc :: SWord8
