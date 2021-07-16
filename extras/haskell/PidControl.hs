{--
  General PID control

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module PidControl where

import AltHoldPid()

data PidFun = AltHoldFun

data PidState = AltHoldState

data PidController = PidController {funPart :: PidFun, statePart :: PidState }

