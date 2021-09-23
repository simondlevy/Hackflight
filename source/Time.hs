{--
  Hackflight time support

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module Time where

import Language.Copilot
import Prelude hiding ((>), (-), (++), (==), (/))

ready :: Stream Float -> Stream Bool

ready freq = (time_sec == time_sec_prev) where

  time_sec_prev = if (time_sec - time_sec_prev') > (1 / freq) then time_sec else time_sec_prev'
  time_sec_prev' = [0] ++ time_sec_prev

time_sec :: Stream Float
time_sec  = extern "copilot_time_sec" Nothing

time_msec :: Stream Word32
time_msec  = extern "copilot_time_msec" Nothing

time_usec :: Stream Word32
time_usec  = extern "copilot_time_usec" Nothing
