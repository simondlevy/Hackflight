{--
  PID controller state

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module PidState

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

newPidController :: PidFun -> PidState -> PidController
newPidController f s = PidController f s
