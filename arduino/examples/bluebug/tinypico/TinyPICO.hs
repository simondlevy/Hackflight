{--
  TinyPICO Support for Bluebug flight controller

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module Bluebug where

import Language.Copilot
import Copilot.Compile.C99

import Time

-- Serial comms
import Serial
import Parser

-- Misc
import Utils

------------------------------------------------------------

spec = do

  -- Get flags for startup, loop
  let (running, starting) = runstate

  -- Run the serial comms parser
  let (msgtype, _, _payindex, _) = parse stream_serial1Available stream_serial1Byte

  -- Check for incoming SET_NORMAL_RC messages from GCS
  --motor_index = if msgtype == 204 && payindex == 1 then stream_serialByte
  --              else motor_index' where motor_index' = [0] ++ motor_index
  --motor_percent = if msgtype == 204 && payindex == 2 then stream_serialByte
  --                else motor_percent' where motor_percent' = [0] ++ motor_percent

  -- Do some stuff at startup
  trigger "stream_startSerial" starting []
  trigger "stream_startSerial1" starting []

  -- Do some other stuff in loop
  -- trigger "stream_updateTime" running []
  trigger "stream_writeLed" running [arg true]
  trigger "stream_serial1Update" running []
  trigger "stream_serial1Read" stream_serial1Available []

  trigger "stream_debug_uint8" running [arg msgtype]

-- Compile the spec
main = reify spec >>= compile "hackflight"
