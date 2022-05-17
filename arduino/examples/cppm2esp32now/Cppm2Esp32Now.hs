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
import MSP
import Messages
import Utils

------------------------------------------------------------

cppm_pin = 4  :: SWord8  
nchan    = 6  :: SWord8  

-- MAC address of receiver peer

rx_mac1 = 0xD8 :: SWord8
rx_mac2 = 0xA0 :: SWord8
rx_mac3 = 0x1D :: SWord8
rx_mac4 = 0x54 :: SWord8
rx_mac5 = 0x87 :: SWord8
rx_mac6 = 0x44 :: SWord8

floatcast :: SWord16 -> SFloat
floatcast w = unsafeCast w

c_receiverGotNewFrame :: SBool
c_receiverGotNewFrame  = extern "receiverGotNewFrame" Nothing

c_receiverChan1 :: SWord16
c_receiverChan1  = extern "receiverChan1" Nothing

c_receiverChan2 :: SWord16
c_receiverChan2  = extern "receiverChan2" Nothing

c_receiverChan3 :: SWord16
c_receiverChan3  = extern "receiverChan3" Nothing

c_receiverChan4 :: SWord16
c_receiverChan4  = extern "receiverChan4" Nothing

c_receiverChan5 :: SWord16
c_receiverChan5  = extern "receiverChan5" Nothing

c_receiverChan6 :: SWord16
c_receiverChan6  = extern "receiverChan6" Nothing

------------------------------------------------------------

spec = do

  -- Get flags for startup, loop
  let (running, starting) = runstate

  -- Do some stuff at startup ----------------------------------------

  trigger "serialStart" starting []

  trigger "cppmStart" starting [ arg cppm_pin, arg nchan ]

  trigger "esp32nowStart" starting []

  trigger "esp32nowAddPeer" starting [  arg rx_mac1
                                      , arg rx_mac2
                                      , arg rx_mac3
                                      , arg rx_mac4
                                      , arg rx_mac5
                                      , arg rx_mac6 ] 

  -- Do some other stuff in loop -------------------------------------

  trigger "cppmUpdate" running []

  trigger "cppmGet" c_receiverGotNewFrame []

  trigger "esp32nowPrepareToSend" running [ 
                                         arg rx_mac1
                                       , arg rx_mac2
                                       , arg rx_mac3
                                       , arg rx_mac4
                                       , arg rx_mac5
                                       , arg rx_mac6
                                       ]

  let msgdir  = 0x3C :: SWord8
  let msgsize = 12   :: SWord8
  let msgtype = 200  :: SWord8
  trigger "commsSend" running [
                              arg msgdir
                            , arg msgsize
                            , arg msgtype
                            , arg $ floatcast c_receiverChan1
                            , arg $ floatcast c_receiverChan2
                            , arg $ floatcast c_receiverChan3
                            , arg $ floatcast c_receiverChan4
                            , arg $ floatcast c_receiverChan5
                            , arg $ floatcast c_receiverChan6
                            ]
-- Compile the spec
main = reify spec >>= compile "hackflight"
