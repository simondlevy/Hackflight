{--
  Hackflight time support

  Copyright(C) 2021 on D.Levy

  MIT License
--}

module Time where

import Language.Copilot

time :: Stream Float
time  = extern "copilot_time" Nothing
