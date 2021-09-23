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

ledPin = 18 :: Stream Word8

m1pin = 13 :: Stream Word8
m2pin = 16 :: Stream Word8
m3pin = 3  :: Stream Word8
m4pin = 11 :: Stream Word8

spec = do

  let (starting, looping, ledOn, motorsReady) = hackflightFull
 
  -- Always update the time
  trigger "copilot_updateTime" true []

  -- Set up the LED during the startup phase
  trigger "copilot_startLed" starting [arg $ ledPin]

  -- Set up serial comms during the startup phase
  trigger "copilot_startSerial" starting []

  -- Set up the motors during the startup phase
  -- trigger "copilot_startBrushedMotors" starting [ arg m1pin
  --                                               , arg m2pin
  --                                               , arg m3pin
  --                                               , arg m4pin ]

  -- Set the LED during the looping phase
  trigger "copilot_setLed" looping [arg $ ledPin, arg ledOn]


  -- Send motor commands periodically during the looping phase
  -- let m = 0 :: Stream Float
  -- trigger "copilot_writeBrushedMotors" (looping && motorsReady)
  --                                      [ arg m1pin,  arg m
  --                                      , arg m2pin , arg m
  --                                      , arg m3pin , arg m
  --                                      , arg m4pin , arg m ]

-- Compile the spec
main = reify spec >>= compile "copilot"
