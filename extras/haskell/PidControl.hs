{--
  PID controller support

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module PidControl

where

import VehicleState
import Demands

data FullPidControl = 

    FullPidControl { fullErrorIntegral :: Double,
                   fullDeltaError1 :: Double,
                   fullDeltaError2 :: Double,
                   fullErrorPrev :: Double }

data PidControl =

     AltHoldState { altErrorIntegral :: Double,
                    altTarget :: Double,
                    altInBand :: Bool }

   | RateState { rateRollState :: FullPidControl,
                 ratePitchState :: FullPidControl }

   | YawState { yawErrorIntegral :: Double }

   | NoState { }

type PidFun = VehicleState -> Demands -> PidControl -> (Demands, PidControl)

data PidController = PidController { pidFun :: PidFun, pidState :: PidControl }

newPidController :: PidFun -> PidControl -> PidController
newPidController f s = PidController f s
