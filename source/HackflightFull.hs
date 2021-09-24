{--
  Hackflight algorithm for microcontrollers

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module HackflightFull where

import Language.Copilot

import Time

data FullStatus = FullStatus {  starting    :: Stream Bool
                              , looping     :: Stream Bool
                              , ledOn       :: Stream Bool
                              , motorsReady :: Stream Bool
                             }

hackflightFull :: FullStatus

hackflightFull = FullStatus starting looping ledOn motorsReady

  where

    -- The looping flag will only be false on startup, so we
    -- can use it for initializtion in our main
    looping = not looping' where looping' = [False] ++ looping
    starting = not looping

    -- This allows us to set the motors periodically
    motorsReady = ready 5 -- 66

    -- Blink LED during first couple of seconds; keep it solid when armed
    ledOn = if micros < 100000 then false 
            else if micros < 2000000 then (mod (div micros 50000) 2 == 0)
            else false -- armed
