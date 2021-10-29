{--
  Hackflight serial parsing

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}
{-# LANGUAGE DataKinds        #-}

module Parser where

import Language.Copilot hiding(xor)
import Prelude hiding((==), (&&), (||), (++), (>), (<), (>=))

import Utils

parse :: SBool -> SWord8 -> (SWord8, SBool, SWord8, SWord8, SWord8, SWord8, SBool)

parse avail byte = (msgtype, sending, index, state, size, crc, checked) where

  state  = if state' == 0 && byte == 36 then 1
      else if state' == 1 && byte == 77 then 2
      else if state' == 2 && (byte == 60 || byte == 62) then 3
      else if state' == 3 then 4
      else if state' == 4 then 5
      else if state' == 5 && msgtype' < 200 then 6
      else if state' == 5 && size' > 1 then 5
      else if state' == 5 && size' == 1 then 6
      else 0

  size = if state == 4 then byte + 2
         else if size' > 0 then size' - 1
         else 0

  msgtype = if state == 5 && msgtype' == 0 then byte
            else if state == 0 then 0
            else msgtype'

  index = if state' < 5 then 0
          else if msgtype >= 200 then index' + 1
          else index'

  crc = if state < 4 then 0 else if state == 6 then crc' else xor crc' byte

  checked = state == 6 && crc == byte

  sending = avail && checked && msgtype' < 200

  -- State variables
  state'     = [0] ++ state :: SWord8
  size'      = [0] ++ size :: SWord8
  msgtype'   = [0] ++ msgtype :: SWord8
  crc'       = [0] ++ crc :: SWord8
  index'     = [0] ++ index :: SWord8
