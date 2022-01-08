{--
  Support for TinyPico flight controller with DSMX receiver input passthru to SBUS output

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module TpDsmx2Sbus where

import Language.Copilot
import Copilot.Compile.C99

import Prelude hiding ((/), (*), (+), (-))

import Time
import Receiver
import Utils

------------------------------------------------------------

dsmx_in_rx_pin = 4  :: SWord8  
dsmx_in_tx_pin = 14 :: SWord8 -- unused

sbus_out_rx_pin = 15 :: SWord8 -- unused 
sbus_out_tx_pin = 27 :: SWord8 

sbus_min = 172  :: SFloat
sbus_max = 1811 :: SFloat

scale :: SFloat -> SFloat
scale val = ((val + 1) / 2) * (sbus_max - sbus_min) + sbus_min

------------------------------------------------------------

spec = do

  -- Get flags for startup, loop
  let (running, starting) = runstate

  -- Do some stuff at startup
  trigger "dsmrxStart" starting [ arg dsmx_in_rx_pin, arg dsmx_in_tx_pin]
  trigger "sbusOutStart" starting [ arg sbus_out_rx_pin, arg sbus_out_tx_pin]

  -- Do some other stuff in loop
  trigger "dsmrxUpdate" running []
  trigger "dsmrxGet" receiverGotNewFrame []
  trigger "sbusWrite" running [  arg $ scale receiverThrottle  
                                      , arg $ scale receiverRoll  
                                      , arg $ scale receiverPitch  
                                      , arg $ scale receiverYaw  
                                      , arg $ scale receiverAux1
                                      , arg $ scale receiverAux2
                                      , arg sbus_min
                                      , arg sbus_min
                                      , arg sbus_min
                                      , arg sbus_min
                                      , arg sbus_min
                                      , arg sbus_min
                                      , arg sbus_min
                                      , arg sbus_min
                                      , arg sbus_min
                                      , arg sbus_min
                                     ]
--}

  -- XXX ignore these varaibles for now
  -- trigger "ignore" running [arg receiverTimedOut]

-- Compile the spec
main = reify spec >>= compile "hackflight"
