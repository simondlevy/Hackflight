{--
  Haskell Copilot support for Hackflight

  Copyright(C) 2021 on D.Levy

  MIT License
--}

module Main where

import Language.Copilot
import Copilot.Compile.C99
import Prelude hiding(not, (==), (<), (>), (++))

import HackflightFull

ledPin :: Stream Word8
ledPin = 18

spec = do

  let (starting, looping, ledOn, motorsReady) = hackflightFull

  trigger "copilot_updateTime" true []

  -- Set up the LED during the starting phase
  trigger "copilot_startLed" starting [arg $ ledPin]

  -- Send the LED using external C function during the looping phase
  trigger "copilot_setLed" looping [arg $ ledPin, arg ledOn]
 
-- Compile the spec
main = reify spec >>= compile "copilot"
