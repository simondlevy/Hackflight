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

-- Passing and returning a tuple allows us to compose PID control functions
type PidFun = (State, SBool, Demands) -> (State, SBool, Demands)
