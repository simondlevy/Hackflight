{--
  Hackflight LED support

  Copyright(C) 2021 on D.Levy

  MIT License
--}

module LED where

import Prelude hiding((<), (&&), (==), div, mod)
import Language.Copilot

import Time(time_msec)

type LedState = Stream Bool

ledState :: LedState

-- Blink LED on startup
ledState = time_msec < 1000 && mod (div time_msec 50) 2 == 0
