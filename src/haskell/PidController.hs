{--
  PID controllers for aerial vehicles

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}


module PidController

where

import Language.Copilot

import State
import Demands

type PidFun = (State, Demands) -> (State, Demands)
