{--
  Hackflight LED support

  Copyright(C) 2021 on D.Levy

  MIT License
--}

module LED where

import Prelude hiding((<), (>), (&&), (||))
import Language.Copilot

import Time(time_sec, time_msec)

type LedState = Stream Bool

ledState :: LedState

-- Blink LED on startup
ledState = time_sec < 1.0 && 
           (  time_sec > 0.00 && time_sec < 0.05
           || time_sec > 0.10 && time_sec < 0.15 
           || time_sec > 0.20 && time_sec < 0.25 
           || time_sec > 0.30 && time_sec < 0.35 
           || time_sec > 0.40 && time_sec < 0.45 
           || time_sec > 0.50 && time_sec < 0.55 
           || time_sec > 0.60 && time_sec < 0.65 
           || time_sec > 0.70 && time_sec < 0.75 
           || time_sec > 0.80 && time_sec < 0.85 
           || time_sec > 0.90 && time_sec < 0.95 
           )
