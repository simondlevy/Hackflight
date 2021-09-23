{-# LANGUAGE RebindableSyntax #-}

module Main where

import Language.Copilot
import Copilot.Compile.C99

import Prelude hiding ((>), (-), (++), (==))

time_sec :: Stream Float
time_sec  = extern "copilot_time_sec" Nothing

ready :: Stream Bool

ready = (time_sec - time_sec_prev)  > 1 where

  time_sec_prev = if (time_sec - time_sec_prev') > 1 then time_sec else time_sec_prev'
  time_sec_prev' = [0] ++ time_sec_prev

times :: (Stream Float, Stream Float)

times = (time_sec, time_sec_prev) where

  time_sec_prev = if (time_sec - time_sec_prev') > 1 then time_sec else time_sec_prev'
  time_sec_prev' = [0] ++ time_sec_prev

spec = do

  let (t, p) = times

  trigger "tick" (t == p) []

-- Compile the spec
main = reify spec >>= compile "timer"

