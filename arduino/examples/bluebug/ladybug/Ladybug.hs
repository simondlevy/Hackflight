{--
  Support for Ladybug flight controller getting RX messages over UART

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module Ladybug where

import Language.Copilot
import Copilot.Compile.C99

import Time
import Serial

-- Misc
import Utils

------------------------------------------------------------

spec = do

  -- Get flags for startup, loop
  let (running, starting) = runstate

  -- Do some stuff at startup
  trigger "stream_serialStart" starting []
  trigger "stream_serial1Start" starting []

  -- Update UART
  trigger "stream_serial1Update" running []

  trigger "stream_debug" (running && stream_serial1Available) [arg stream_serial1Byte]

-- Compile the spec
main = reify spec >>= compile "hackflight"
