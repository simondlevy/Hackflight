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

  let (looping, ledState) = hackflightFull

  trigger "copilot_updateClock" true []

  trigger "copilot_startLed" (not looping) [arg $ ledPin]

  -- Send the LED using external C function during the looping phase
  trigger "copilot_setLed" looping [arg $ ledPin, arg ledState]
 
-- Compile the spec
main = reify spec >>= compile "copilot"
