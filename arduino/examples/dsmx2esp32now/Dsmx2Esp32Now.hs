{--
  Support for flight controller with DSMX receiver input to ESP32NOW output

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module Dsmx2Esp32Now where

import Language.Copilot
import Copilot.Compile.C99

import Prelude hiding ((/), (*), (+), (-), (==), (++))

import Time
import Receiver
import MSP
import Messages
import Utils

------------------------------------------------------------

dsmx_in_rx_pin = 4  :: SWord8  
dsmx_in_tx_pin = 14 :: SWord8 -- unused

rx_mac1 = 0x98 :: SWord8
rx_mac2 = 0xCD :: SWord8
rx_mac3 = 0xAC :: SWord8
rx_mac4 = 0xD3 :: SWord8
rx_mac5 = 0x42 :: SWord8
rx_mac6 = 0xE0 :: SWord8

------------------------------------------------------------

spec = do

  -- Get flags for startup, loop
  let (running, starting) = runstate

  -- Do some stuff at startup ----------------------------------------

  trigger "serialStart" starting []

  trigger "dsmrxStart" starting [ arg dsmx_in_rx_pin, arg dsmx_in_tx_pin]

  trigger "esp32nowStart" starting []

  trigger "esp32nowAddPeer" starting [  arg rx_mac1
                                      , arg rx_mac2
                                      , arg rx_mac3
                                      , arg rx_mac4
                                      , arg rx_mac5
                                      , arg rx_mac6 ] 

  let message = Messages.rxmessage Receiver.c_receiverThrottle
                                   Receiver.c_receiverRoll
                                   Receiver.c_receiverPitch
                                   Receiver.c_receiverYaw
                                   Receiver.c_receiverAux1
                                   Receiver.c_receiverAux2

  -- Do some other stuff in loop -------------------------------------

  trigger "dsmrxUpdate" running []

  trigger "dsmrxGet" c_receiverGotNewFrame []

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
