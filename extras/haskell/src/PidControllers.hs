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
                                     pidState :: PidState }

pidUpdate :: VehicleState -> (Demands, PidController) -> 
             (Demands, PidController)

pidUpdate vehicleState (demands, pidController) = 

    let pfun = pidFun pidController

        (demands', pstate) = pfun vehicleState demands (pidState pidController)

        pidController' = PidController pfun pstate

    in (demands', pidController')
