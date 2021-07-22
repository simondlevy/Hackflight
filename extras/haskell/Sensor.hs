{--
  Sensor support

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module Sensor

where

import VehicleState

-- type Sensor = IO () -> VehicleState -> VehicleState

type Sensor = VehicleState -> VehicleState

runSensors :: [Sensor] -> vehicleState -> vehicleState

runSensors _sensors vehicleState = vehicleState

{--

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

--}
