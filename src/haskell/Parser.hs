{--
  Hackflight serial parsing

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}
{-# LANGUAGE DataKinds        #-}

module Parser where

import Language.Copilot hiding(xor)
import Prelude hiding((==), (>=), (<=), (&&), (||), (++), not)

import Serial
import Utils

parse :: SBool -> SWord8 -> (SBool, SBool, SWord8, SWord8)

parse avail byte = (sending, receiving, pindex, msgtype) where

  -- Payload handling
  input_size = if state == 3 then byte else input_size'
  payload_index = if state == 5 then payload_index' + 1 else 0

  receiving = msgtype >= 200 && state == 5 && payload_index <= input_size

  -- Command acquisition function
  msgtype = if state == 4 then byte else msgtype'

  -- Checksum transition function
  crc_in = if state == 3 then byte
      else if state == 4  then  xor crc_in byte 
      else if receiving then xor crc_in byte
      else if state == 5  then crc_in'
      else 0

  -- Parser state transition function
  state  = if state' == 0 && byte == 36 then 1
      else if state' == 1 && byte == 77 then 2
      else if state' == 2 && (byte == 60 || byte == 62) then 3
      else if state' == 3 then 4
      else if state' == 4 then 5
      else if state' == 5 && receiving then 5
      else if state' == 5 then 0
      else state' :: SWord8

  incoming = msgtype >= 200

  sending = avail && state == 0 && crc_in == byte && not incoming

  pindex = if receiving then payload_index - 1 else 0

  -- State variables
  input_size'    = [0] ++ input_size
  state'         = [0] ++ state
  payload_index' = [0] ++ payload_index
  crc_in'        = [0] ++ crc_in
  msgtype'       = [0] ++ msgtype

