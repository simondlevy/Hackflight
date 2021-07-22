{--
  Debugging support

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module Debugging

where

import Text.Printf
import Debug.Trace

debug :: Double -> Double
debug val = trace (printf "%+3.3f" val) val
