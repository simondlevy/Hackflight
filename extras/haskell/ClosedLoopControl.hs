{--
  Hackflight closed-loop control algorithm

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module ClosedLoopControl(closedLoop)
where

import VehicleState
import Demands
import PidControl(PidController, newPidController, pidFun, pidState)

closedLoop :: Demands ->
              VehicleState ->
              [PidController] ->
              (Demands, [PidController])

closedLoop demands vehicleState oldPidControllers =

    closedLoopHelper demands vehicleState oldPidControllers []
  

closedLoopHelper :: Demands ->
                    VehicleState ->
                    [PidController] ->
                    [PidController] ->
                    (Demands, [PidController])

-- Base case: return final demands and PID controllers
closedLoopHelper demands  _ [] newPidControllers = (demands, newPidControllers)

-- Recursive case: apply current PID controller to demands to get new demands
-- and PID state; then recur on remaining PID controllers
closedLoopHelper demands vehicleState oldPidControllers newPidControllers =

    let oldPidController = head oldPidControllers
   
        pfun = pidFun oldPidController

        pstate = pidState oldPidController

        (newDemands, newPstate) = pfun vehicleState demands pstate

        newPid = newPidController pfun newPstate

    in closedLoopHelper newDemands
                        vehicleState
                        (tail oldPidControllers)
                        (newPidControllers ++ [newPid])
