{--
  TinyPICO Support for Bluebug flight controller

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module TinyPICO where

import Language.Copilot
import Copilot.Compile.C99

import Time

-- Comms
import Bluetooth
import Serial
import Parser

-- Misc
import Utils

------------------------------------------------------------

spec = do

  -- Get flags for startup, loop
  let (running, starting) = runstate

  -- Shorthand
  let avail = stream_bluetoothAvailable
  let byte = stream_bluetoothByte

  -- Run the serial comms parser
  let (msgtype, _, payindex, checked) = parse avail byte

  let chan1 = if payindex == 1 then cast byte
              else if payindex == 2 then chan1' .|. ((cast byte) .<<. s8)
              else chan1'
              where chan1' = [0] ++ chan1 :: SWord16
                    s8 = 8 :: SWord8

  -- Do some stuff at startup
  trigger "stream_startSerial" starting []
  trigger "stream_startBluetooth" starting []

  -- Do some other stuff in loop
  -- trigger "stream_updateTime" running []
  trigger "stream_bluetoothUpdate" running []
  trigger "stream_bluetoothRead" (running && stream_bluetoothAvailable) []


  -- trigger "stream_debug" checked [arg chan1]
  -- trigger "stream_debug" (payindex == 2) [arg chan1]
  trigger "stream_debug" (running && avail) [arg byte]

-- Compile the spec
main = reify spec >>= compile "hackflight"

