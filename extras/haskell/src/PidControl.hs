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

newPidController :: PidFun -> PidState -> PidController
newPidController f s = PidController f s

--------------------------------------------------------------------------------

runClosedLoop :: Demands ->
                 VehicleState ->
                 [PidController] ->
                 (Demands, [PidController])

runClosedLoop demands vehicleState pidControllers =

    closedLoopHelper demands pidControllers []

    where

        -- Base case: ignore vehicle state and return new demands and new PID
        -- controllers
        closedLoopHelper newDemands  [] newPidControllers =
            (newDemands, newPidControllers)

        -- Recursive case: apply current PID controller to demands to get new
        -- demands and PID state; then recur on remaining PID controllers
        closedLoopHelper oldDemands oldPidControllers newPidControllers =

            let oldPidController = head oldPidControllers
           
                pfun = pidFun oldPidController

                pstate = pidState oldPidController

                (newDemands, newPstate) = pfun vehicleState oldDemands pstate

                newPid = newPidController pfun newPstate

            in closedLoopHelper newDemands
                                (tail oldPidControllers)
                                (newPidControllers ++ [newPid])
