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

data ParserState = ParserState { sending   :: SBool
                               , receiving :: SBool
                               , index     :: SWord8
                               , msgtype   :: SWord8 }

parse :: SBool -> SWord8 -> ParserState

parse avail byte = ParserState sending receiving pindex msgtype where

  -- Parser state transition function
  state  = if state' == 0 && byte == 36 then 1
      else if state' == 1 && byte == 77 then 2
      else if state' == 2 && (byte == 60 || byte == 62) then 3
      else if state' == 3 then 4
      else if state' == 4 then 5
      else if state' == 5 && receiving' then 5
      else if state' == 5 then 0
      else state'

  size = if state == 3 then byte else size'

  index = if state == 5 then index' + 1 else 0

  msgtype = if state == 4 then byte else msgtype'

  receiving = if msgtype >= 200 && state' == 5 && index <= size
              then true
              else receiving'

  incoming = size > 0

  -- Checksum transition function
  crc = if state == 3 then byte
      else if state == 4  then  xor crc' byte 
      else if receiving then xor crc' byte
      else if state == 5  then crc'
      else 0

  sending = avail && state == 0 && crc == byte && not incoming

  pindex = if receiving then index - 1 else 0

  -- State variables
  state'     = [0] ++ state :: SWord8
  size'      = [0] ++ size
  index'     = [0] ++ index :: SWord8
  msgtype'   = [0] ++ msgtype :: SWord8
  crc'       = [0] ++ crc :: SWord8
  receiving' = [False] ++ receiving
