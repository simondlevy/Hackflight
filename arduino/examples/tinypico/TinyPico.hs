{--
  Support for TinyPico flight controller with DSMX receiver input passthru to SBUS output

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module TinyPico where

import Language.Copilot
import Copilot.Compile.C99

import Time
import Receiver
import Utils

------------------------------------------------------------

dsmx_in_rx_pin = 4 :: SWord8  
dsmx_in_tx_pin = 14 :: SWord8 -- unused

sbus_out_rx_pin = 15 :: SWord8 -- unused 
sbus_out_tx_pin = 27 :: SWord8 

------------------------------------------------------------

spec = do

  -- Get flags for startup, loop
  let (running, starting) = runstate

  -- Do some stuff at startup
  trigger "stream_startDsmrx" starting [ arg dsmx_in_rx_pin, arg dsmx_in_tx_pin]
  trigger "stream_startSbusOut" starting [ arg sbus_out_rx_pin, arg sbus_out_tx_pin]

  -- Do some other stuff in loop
  trigger "stream_updateDsmrx" running []
  trigger "stream_getDsmrx" receiverGotNewFrame []
  trigger "stream_writeSbus" running [  arg receiverThrottle
                                      , arg receiverRoll
                                      , arg receiverPitch
                                      , arg receiverYaw
                                      , arg receiverAux1
                                     ]

  -- XXX ignore these varaibles for now
  trigger "stream_ignore" running [arg receiverTimedOut, arg receiverAux2]


-- Compile the spec
main = reify spec >>= compile "hackflight"
