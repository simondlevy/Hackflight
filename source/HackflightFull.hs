{--
  Hackflight algorithm with serial comms

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module HackflightFull where

import Language.Copilot

import Time

hackflightFull :: (Stream Bool, Stream Bool, Stream Bool)

hackflightFull = (looping, ledOn, motorsReady)

  where

    -- The looping flag will only be false on startup, so we
    -- can use it for initializtion in our main
    looping = not looping' where looping' = [False] ++ looping

    -- This allows to set the motors periodically
    motorsReady = ready 2

    -- Blink LED during first couple of seconds; keep it solid when armed
    ledOn = if time_msec < 2000 then (mod (div time_msec 50) 2 == 0) else false -- armed
