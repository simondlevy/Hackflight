{--
  Debugging support

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module Debugging

where

import Text.Printf
import Debug.Trace

debug :: [Char] -> Double -> Double
debug comment value = trace (printf "%s %+3.3f" comment value) value
