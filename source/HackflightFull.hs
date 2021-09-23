{--
  Hackflight algorithm with serial comms

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module HackflightFull where

import Language.Copilot

import Prelude hiding((||), (++), (<), (>), (&&), (==), div, mod, not)

import Time(time_msec)

hackflightFull :: (Stream Bool, Stream Bool)

hackflightFull = (looping, led)

  where

    looping = not looping' where looping' = [False] ++ looping

    -- Blink LED on startup, keep solid when armed
    led = if time_msec < 2000 then (mod (div time_msec 50) 2 == 0) else false -- armed
