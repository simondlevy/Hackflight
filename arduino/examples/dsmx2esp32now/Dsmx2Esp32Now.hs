{--
  DSMX receiver input to ESP32NOW output

  Copyright(C) 2022 on D.Levy

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

-- Pins for DSMX receiver input
dsmx_in_rx_pin = 4  :: SWord8  
dsmx_in_tx_pin = 14 :: SWord8 -- unused

-- MAC address of receiver peer
rx_mac1 = 0x98 :: SWord8
rx_mac2 = 0xCD :: SWord8
rx_mac3 = 0xAC :: SWord8
rx_mac4 = 0xD3 :: SWord8
rx_mac5 = 0x42 :: SWord8
rx_mac6 = 0xE0 :: SWord8

floatcast :: SWord16 -> SFloat
floatcast w = unsafeCast w

------------------------------------------------------------

spec = do

  -- Get flags for startup, loop
  let (running, starting) = runstate

  -- Do some stuff at startup ----------------------------------------

  trigger "serialStart" starting []

  trigger "dsmrxStart" starting [ arg dsmx_in_rx_pin, arg dsmx_in_tx_pin]

{--
  trigger "esp32nowStart" starting []

  trigger "esp32nowAddPeer" starting [  arg rx_mac1
                                      , arg rx_mac2
                                      , arg rx_mac3
                                      , arg rx_mac4
                                      , arg rx_mac5
                                      , arg rx_mac6 ] 
--}
  -- Do some other stuff in loop -------------------------------------

  trigger "dsmrxUpdate" running []

  trigger "dsmrxGet" c_receiverGotNewFrame []

  trigger "dump" running [  arg c_receiverThrottle
                          , arg c_receiverRoll
                          , arg c_receiverPitch
                          , arg c_receiverYaw
                          , arg c_receiverAux1
                          , arg c_receiverAux2 ]
 

{--
  trigger "esp32nowPrepareToSend" true [ 
                                         arg rx_mac1
                                       , arg rx_mac2
                                       , arg rx_mac3
                                       , arg rx_mac4
                                       , arg rx_mac5
                                       , arg rx_mac6
                                       ]

  let msgdir  = 0x3E :: SWord8
  let msgsize = 12   :: SWord8
  let msgtype = 0x3E :: SWord8

  trigger "commsSend" running [
                              arg msgdir
                            , arg msgsize
                            , arg msgtype
                            , arg $ floatcast c_receiverThrottle
                            , arg $ floatcast c_receiverRoll
                            , arg $ floatcast c_receiverPitch
                            , arg $ floatcast c_receiverYaw
                            , arg $ floatcast c_receiverAux1
                            , arg $ floatcast c_receiverAux2
                            ]
--}

-- Compile the spec
main = reify spec >>= compile "hackflight"
