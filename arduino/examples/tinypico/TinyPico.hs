{--
  Support for TinyPico flight controller with DSMX receiver input passthru to SBUS output

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module TinyPico where

import Language.Copilot
import Copilot.Compile.C99

import Time(runstate)

import Receiver(receiverThrottle, receiverRoll, receiverPitch, receiverYaw, receiverAux1)

------------------------------------------------------------

spec = do

  -- Get flags for startup, loop
  let (running, starting) = runstate

  -- Do some stuff at startup
  trigger "stream_startDsmrx" starting []
  trigger "stream_startSbusOut" starting []

  -- Do some other stuff in loop
  trigger "stream_updateDsmrx" running []
  trigger "stream_writeSbus" running [  arg receiverThrottle
                                      , arg receiverRoll
                                      , arg receiverPitch
                                      , arg receiverYaw
                                      , arg receiverAux1
                                     ]


-- Compile the spec
main = reify spec >>= compile "hackflight"
