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

  -- Do some stuff at startup
  trigger "stream_startSerial" starting []

  -- Do some other stuff in loop
  trigger "stream_updateTime" running []
  trigger "stream_writeLed" running []
  trigger "stream_serialUpdate" running []
  trigger "stream_serial1Read" stream_serialAvailable []

-- Compile the spec
main = reify spec >>= compile "hackflight"
