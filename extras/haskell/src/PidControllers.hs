{--
  PID controllers for aerial vehicles

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module PidControllers

where

import FullPidController(FullPidState)
import VehicleState
import Demands

data PidState =

     AltHoldState { altErrorIntegral :: Double,
                    altTarget :: Double,
                    altInBand :: Bool }

   | RateState { rateRollState :: FullPidState,
                 ratePitchState :: FullPidState }

   | YawState { yawErrorIntegral :: Double }

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
