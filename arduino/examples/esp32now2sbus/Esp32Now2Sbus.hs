{--
  Support for flight controller with ESP32NOW receiver input to SBUS output

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module Esp32Now2Sbus where

import Language.Copilot
import Copilot.Compile.C99

import Prelude hiding ((/), (*), (+), (-), (==), (++))

import Time
import Receiver
import MSP
import Messages
import Utils

------------------------------------------------------------

tx_mac1 = 0x98 :: SWord8
tx_mac2 = 0xCD :: SWord8
tx_mac3 = 0xAC :: SWord8
tx_mac4 = 0xD3 :: SWord8
tx_mac5 = 0x42 :: SWord8
tx_mac6 = 0x3C :: SWord8

------------------------------------------------------------

spec = do

  -- Get flags for startup, loop
  let (running, starting) = runstate

  -- Do some stuff at startup ----------------------------------------

  trigger "esp32nowStart" starting []

  trigger "esp32nowAddPeer" starting [  arg tx_mac1
                                      , arg tx_mac2
                                      , arg tx_mac3
                                      , arg tx_mac4
                                      , arg tx_mac5
                                      , arg tx_mac6 ] 

  -- Do some other stuff in loop -------------------------------------

  trigger "commsSend" true [ 
                             arg $ direction message
                           , arg $ paysize message
                           , arg $ msgtype message
                           , arg $ v1 message
                           , arg $ v2 message
                           , arg $ v3 message
                           , arg $ v4 message
                           , arg $ v5 message
                           , arg $ v6 message
                           ]

-- Compile the spec
main = reify spec >>= compile "hackflight"
