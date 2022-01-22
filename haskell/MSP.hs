{--
  General code for MSP messages

  See https://www.hamishmb.com/multiwii/wiki/index.php?title=Multiwii_Serial_Protocol

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}
{-# LANGUAGE DataKinds        #-}

module MSP where

import Language.Copilot hiding(xor)
import Prelude hiding((==), (&&), (||), (++), (>), (<), not)

import Messages
import State
import Utils

--------------------------------------------------------

mkresponse ::  SWord8  -- size
            -> SWord8  -- type
            -> SFloat  -- v1
            -> SFloat  -- v2
            -> SFloat  -- v3
            -> SFloat  -- v4
            -> SFloat  -- v5
            -> SFloat  -- v6
            -> Message

mkresponse size msgtype v1 v2 v3 v4 v5 v6 =
  Message 0x3E size msgtype v1 v2 v3 v4 v5 v6 0 0 0 0 0 0 0 0 0 0 0 0

--------------------------------------------------------

mkcommand ::   SWord8  -- size
            -> SWord8  -- type
            -> SFloat  -- v1
            -> SFloat  -- v2
            -> SFloat  -- v3
            -> SFloat  -- v4
            -> SFloat  -- v5
            -> SFloat  -- v6
            -> Message

mkcommand size msgtype v1 v2 v3 v4 v5 v6 =
  Message 0x3C size msgtype v1 v2 v3 v4 v5 v6 0 0 0 0 0 0 0 0 0 0 0 0

--------------------------------------------------------

reply :: SWord8 -> State -> Message

reply msgtype vstate =

  let (paysize, v1, v2, v3, v4, v5, v6) = Messages.payload msgtype vstate

  in mkresponse paysize msgtype v1 v2 v3 v4 v5 v6

--------------------------------------------------------

parse :: SWord8 -> (SWord8, SWord8, SBool, SBool)

parse byte = (msgtype, payindex, checked, isrequest) where

  isrequest = msgtype' < 200

  -- parser state-transition function
  state  = if byte == 36 then 1
      else if state' == 1 && byte == 77 then 2
      else if state' == 2 && (byte == 60 || byte == 62) then 3
      else if state' == 3 then 4
      else if state' == 4 then 5
      else if state' == 5 && isrequest then 6
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
             else if not isrequest then payindex' + 1
             else payindex'

  crc = if state < 4 then 0 else if state == 6 then crc' else xor crc' byte

  checked = state == 6 && crc == byte

  -- State variables
  state'     = [0] ++ state :: SWord8
  size'      = [0] ++ size
  msgtype'   = [0] ++ msgtype
  crc'       = [0] ++ crc
  payindex'  = [0] ++ payindex :: SWord8
