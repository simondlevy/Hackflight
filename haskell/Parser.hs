{--
  Hackflight serial parsing

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}
{-# LANGUAGE DataKinds        #-}

module Parser where

import Language.Copilot hiding(xor)
import Prelude hiding((==), (&&), (||), (++), (>), (<), not)

import MSP
import Messages
import State
import Utils

reply :: SWord8 -> State -> Message

reply msgtype vstate =

  let (paysize, v1, v2, v3, v4, v5, v6) = payload msgtype vstate

  in MSP.mkresponse paysize msgtype v1 v2 v3 v4 v5 v6

parse :: SBool -> SWord8 -> (SWord8, SBool, SWord8, SBool)

parse avail byte = (msgtype, sending, payindex, checked) where

  iscommand = msgtype' < 200

  state  = if byte == 36 then 1
      else if state' == 1 && byte == 77 then 2
      else if state' == 2 && (byte == 60 || byte == 62) then 3
      else if state' == 3 then 4
      else if state' == 4 then 5
      else if state' == 5 && iscommand then 6
      else if state' == 5 && size' > 1 then 5
      else if state' == 5 && size' == 1 then 6
      else 0

  size = if state == 4 then byte + 2
         else if size' > 0 then size' - 1
         else 0

  msgtype = if state == 4 then 0
            else if state == 5 && msgtype' == 0 then byte
            else msgtype'

  payindex = if state' < 5 then 0
             else if not iscommand then payindex' + 1
             else payindex'

  crc = if state < 4 then 0 else if state == 6 then crc' else xor crc' byte

  checked = state == 6 && crc == byte

  sending = avail && checked && iscommand

  -- State variables
  state'     = [0] ++ state :: SWord8
  size'      = [0] ++ size
  msgtype'   = [0] ++ msgtype
  crc'       = [0] ++ crc
  payindex'  = [0] ++ payindex :: SWord8
