{--
  TinyPICO Support for Bluebug flight controller

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module Bluebug where

import Language.Copilot
import Copilot.Compile.C99
import Copilot.Language.Operators.BitWise((.<<.), (.|.))

import Time

-- Comms
import Bluetooth
import Serial
import Parser

-- Misc
import Utils

s8 = 8 :: SWord8

------------------------------------------------------------

spec = do

  -- Get flags for startup, loop
  let (running, starting) = runstate

  -- Shorthand
  let byte = stream_bluetoothByte

  -- Run the serial comms parser
  let (msgtype, _, payindex, checked) = parse stream_bluetoothAvailable byte

  let chan1 = if payindex == 1 then cast byte
              else if payindex == 2 then chan1' .|. ((cast byte) .<<. s8)
              else chan1'
              where chan1' = [0] ++ chan1 :: SWord16

  -- Check for incoming SET_NORMAL_RC messages from GCS
  --motor_index = if msgtype == 204 && payindex == 1 then stream_serialByte
  --              else motor_index' where motor_index' = [0] ++ motor_index
  --motor_percent = if msgtype == 204 && payindex == 2 then stream_serialByte
  --                else motor_percent' where motor_percent' = [0] ++ motor_percent

  -- Do some stuff at startup
  trigger "stream_startSerial" starting []
  trigger "stream_startBluetooth" starting []

  -- Do some other stuff in loop
  -- trigger "stream_updateTime" running []
  trigger "stream_writeLed" running [arg true]
  trigger "stream_bluetoothUpdate" running []
  trigger "stream_bluetoothRead" stream_bluetoothAvailable []

  -- trigger "stream_debug" checked [arg chan1]
  trigger "stream_debug" (payindex == 2) [arg chan1]

-- Compile the spec
main = reify spec >>= compile "hackflight"

