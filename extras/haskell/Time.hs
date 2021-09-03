{--
  Hackflight time support

  Copyright(C) 2021 on D.Levy

  MIT License
--}

module Time where

import Language.Copilot

time_sec :: Stream Float
time_sec  = extern "copilot_time_sec" Nothing

time_msec :: Stream Word32
time_msec  = extern "copilot_time_msec" Nothing

time_usec :: Stream Word32
time_usec  = extern "copilot_time_usec" Nothing
