{--
  Hackflight serial parsing

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}
{-# LANGUAGE DataKinds        #-}

module Parser where

import Language.Copilot hiding(xor)
import Prelude hiding((==), (&&), (||), (++), (>))

import Utils

parse1 :: SBool -> SWord8 -> (SWord8, SWord8, SWord8, SWord8, SBool, SBool)

parse1 avail byte = (size, msgtype, crc, state, sending, receiving) where

  state  = if state' == 0 && byte == 36 then 1
      else if state' == 1 && byte == 77 then 2
      else if state' == 2 && (byte == 60 || byte == 62) then 3
      else if state' == 3 then 4
      else if state' == 4 then 5
      else if state' == 5 then 6
      else if state' == 6 then 0
      else state'

  size = if state == 4 then byte else size'
  msgtype = if state == 5 then byte else msgtype'
  receiving = if state == 4 then size > 0 else if state == 5 then receiving' else false
  crc = if state == 6 then byte else crc'

  sending = avail && state == 6 && crc == byte && size == 0

  -- State variables
  state'     = [0] ++ state :: SWord8
  size'      = [0] ++ size :: SWord8
  msgtype'   = [0] ++ msgtype :: SWord8
  receiving' = [False] ++ receiving
  crc'       = [0] ++ crc :: SWord8











