{-# LANGUAGE RebindableSyntax #-}

module Main where

import Language.Copilot
import Copilot.Compile.C99

import Prelude hiding ((>), (-), (++), (==), div)

time_sec :: Stream Float
time_sec  = extern "copilot_time_sec" Nothing

ready :: Stream Float -> Stream Bool

ready period = (time_sec == time_sec_prev) where

  time_sec_prev = if (time_sec - time_sec_prev') > period then time_sec else time_sec_prev'
  time_sec_prev' = [0] ++ time_sec_prev

spec = do

  trigger "tick" (ready 1) []

-- Compile the spec
main = reify spec >>= compile "timer"

