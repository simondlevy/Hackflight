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

data PidState =

     AltHoldState { altErrorIntegral :: Stream Double,
                    altTarget :: Stream Double,
                    altInBand :: Stream Bool }

   | RateState { rateRollState :: FullPidState,
                 ratePitchState :: FullPidState }

   | YawState { yawErrorIntegral :: Stream Double }

   | NoState { }

type PidFun = VehicleState -> Demands -> PidState -> (Demands, PidState)

data PidController = PidController { pidFun :: PidFun,
                                     pidState :: PidState,
                                     pidDemands :: Demands }

makePidController :: PidFun -> PidState -> PidController

makePidController pidFun' pidState' =
    PidController pidFun' pidState' zeroDemands

pidUpdate :: VehicleState -> PidController -> PidController

pidUpdate vehicleState pidController = 

    let pidFun' = pidFun pidController

        (demands', pidState') = pidFun' vehicleState
                                        (pidDemands pidController)
                                        (pidState pidController)

    in PidController pidFun' pidState' demands'
