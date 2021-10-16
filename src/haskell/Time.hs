{--
  Hackflight time support

  Copyright(C) 2021 on D.Levy

  MIT License
--}

module Time where

import Language.Copilot

import Utils

micros :: SWord32
micros  = extern "stream_micros" Nothing
