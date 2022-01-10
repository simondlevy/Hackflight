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
import Parser
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

  -- Do some stuff at startup


  trigger "serialStart" starting []

  --trigger "dsmrxStart" starting [ arg dsmx_in_rx_pin, arg dsmx_in_tx_pin]

  trigger "esp32nowStart" starting []

  trigger "esp32nowAddPeer" starting [  arg rx_mac1
                                      , arg rx_mac2
                                      , arg rx_mac3
                                      , arg rx_mac4
                                      , arg rx_mac5
                                      , arg rx_mac6 ] 

  let message = rxmessage c_receiverThrottle
                          c_receiverRoll
                          c_receiverPitch
                          c_receiverYaw
                          c_receiverAux1
                          c_receiverAux2

  -- Do some other stuff in loop
  --trigger "dsmrxUpdate" running []
  --trigger "dsmrxGet" receiverGotNewFrame []
  trigger "esp32nowSend" running [
                                   arg $ hdr0 message
                                 , arg $ hdr1 message
                                 , arg $ hdr2 message
                                 , arg $ outsize message
                                 , arg $ msgtype message
                                 , arg $ crc message
                                 , arg $ paysize message
                                 , arg $ val00 message
                                 , arg $ val01 message
                                 , arg $ val02 message
                                 , arg $ val03 message
                                 , arg $ val04 message
                                 , arg $ val05 message
                                 ]

-- Compile the spec
main = reify spec >>= compile "hackflight"
