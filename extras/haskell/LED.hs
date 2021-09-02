{--
  Hackflight LED support

  Copyright(C) 2021 on D.Levy

  MIT License
--}

module LED where

import Prelude hiding((<), (>), (&&), (||))
import Language.Copilot

import Time

type LedState = Stream Bool

ledState :: LedState

-- Blink LED on startup
ledState = time < 1.0 && 
           (  time > 0.00 && time < 0.05
           || time > 0.10 && time < 0.15 
           || time > 0.20 && time < 0.25 
           || time > 0.30 && time < 0.35 
           || time > 0.40 && time < 0.45 
           || time > 0.50 && time < 0.55 
           || time > 0.60 && time < 0.65 
           || time > 0.70 && time < 0.75 
           || time > 0.80 && time < 0.85 
           || time > 0.90 && time < 0.95 
           )
