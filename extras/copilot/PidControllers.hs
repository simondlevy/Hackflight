{--
  PID controllers for aerial vehicles

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}
{-# LANGUAGE DataKinds        #-}

module PidControllers

where

import Language.Copilot
import Copilot.Compile.C99

import FullPidController(FullPidState)
import VehicleState
import Demands

type PidFun = VehicleState -> Demands -> Demands

data PidController = PidController { pidFun :: PidFun, pidDemands :: Demands }

makePidController :: PidFun -> PidController

makePidController pidFun' = PidController pidFun' zeroDemands

pidUpdate :: VehicleState -> PidController -> PidController

pidUpdate vehicleState pidController = 

    let pidFun' = pidFun pidController

        demands' = pidFun' vehicleState (pidDemands pidController)

    in PidController pidFun' demands'
