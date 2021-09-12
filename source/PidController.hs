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

type PidFun = State -> Demands -> Demands

data PidController = PidController { pidFun :: PidFun, pidDemands :: Demands }

makePidController :: PidFun -> PidController

makePidController pidFun' = PidController pidFun' (Demands 0 0 0 0)

pidUpdate :: State -> PidController -> PidController

pidUpdate state pidController = 

    let pidFun' = pidFun pidController

        demands' = pidFun' state (pidDemands pidController)

    in PidController pidFun' demands'
