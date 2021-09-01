{--
  Hackflight LED support

  Copyright(C) 2021 on D.Levy

  MIT License
--}

module LED where

import Prelude hiding((<))
import Language.Copilot
import Time

type LedState = Stream Bool

ledState :: LedState

-- Blink LED on startup
ledState = time < 3.0

