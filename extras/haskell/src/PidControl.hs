{--
  Closed-loop controller support

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module PidControl

where

import VehicleState
import Demands

data FullPidState = 

    FullPidState { fullErrorIntegral :: Double,
                   fullDeltaError1 :: Double,
                   fullDeltaError2 :: Double,
                   fullErrorPrev :: Double }

data PidState =

     AltHoldState { altErrorIntegral :: Double,
                    altTarget :: Double,
                    altInBand :: Bool }

   | RateState { rateRollState :: FullPidState,
                 ratePitchState :: FullPidState }

   | YawState { yawErrorIntegral :: Double }

   | NoState { }

type PidFun = VehicleState -> Demands -> PidState -> (Demands, PidState)

data PidController = PidController { pidFun :: PidFun, pidState :: PidState }

pidUpdate :: VehicleState -> (Demands, PidController) -> 
             (Demands, PidController)

pidUpdate vehicleState (demands, pidController) = 

    let pfun = pidFun pidController

        (newDemands, newPstate) = pfun vehicleState
                                  demands
                                  (pidState pidController)

        newPid = PidController pfun newPstate

    in (newDemands, newPid)


