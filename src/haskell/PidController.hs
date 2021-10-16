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
import Utils

type PidFun = (State, Demands, SBool) -> Demands
