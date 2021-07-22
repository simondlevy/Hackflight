{--
  Closed-loop controller support

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module ClosedLoopControl

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

closedLoop :: Demands ->
              VehicleState ->
              [PidController] ->
              (Demands, [PidController])

closedLoop demands vehicleState pidControllers =

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
