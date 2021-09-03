{--
  Hackflight LED support

  Copyright(C) 2021 on D.Levy

  MIT License
--}

module LED where

import Prelude hiding((<), (>), (&&), (||))
import Language.Copilot

import Time(time_msec)

type LedState = Stream Bool

ledState :: LedState

-- Blink LED on startup
ledState = time_msec < 1000 && 
           (  time_msec > 0 && time_msec < 50
           || time_msec > 100 && time_msec < 150
           || time_msec > 200 && time_msec < 250 
           || time_msec > 300 && time_msec < 350 
           || time_msec > 400 && time_msec < 450 
           || time_msec > 500 && time_msec < 550 
           || time_msec > 600 && time_msec < 650 
           || time_msec > 700 && time_msec < 750 
           || time_msec > 800 && time_msec < 850 
           || time_msec > 900 && time_msec < 950 
           )
