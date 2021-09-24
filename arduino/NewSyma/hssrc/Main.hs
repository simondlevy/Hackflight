{--
  Haskell Copilot support for Hackflight

  Copyright(C) 2021 on D.Levy

  MIT License
--}

module Main where

import Language.Copilot
import Copilot.Compile.C99
import Prelude hiding((&&))

import HackflightFull
import Receiver
import Time

ledPin = 18 :: Stream Word8

m1pin = 13 :: Stream Word8
m2pin = 16 :: Stream Word8
m3pin = 3  :: Stream Word8
m4pin = 11 :: Stream Word8

receiver = makeReceiver 4.0


spec = do

  let status = hackflightFull -- receiver sensors pidControllers mixer
 
  -- Set up serial comms during the startup phase
  trigger "copilot_startSerial" (starting status) []

  -- Set up the LED during the startup phase
  trigger "copilot_startLed" (starting status) [arg $ ledPin]

  -- Update the time during the looping phase
  trigger "copilot_updateTime" true []

  -- Set the LED during the looping phase
  trigger "copilot_setLed" (looping status) [arg $ ledPin, arg (ledOn status)]

  trigger "copilot_debug" (motorsReady status) []

-- Compile the spec
main = reify spec >>= compile "copilot"
