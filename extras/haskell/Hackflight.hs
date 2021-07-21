{--
  Hackflight core algorithm

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module Hackflight(HackflightFun, hackflightFun)
where

import VehicleState
import Demands
import PidControllers(PidController, newPidController, pidFun, pidState)
import Mixer(Mixer, Motors)

type HackflightFun = Demands ->
                     VehicleState ->
                     Mixer ->
                     [PidController] ->
                     (Motors, [PidController])

hackflightFun :: HackflightFun

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
                        (newPid:newPidControllers)

hackflightFun demands vehicleState mixer pidControllers =

    let (newDemands, newPidControllers) = closedLoopHelper demands
                                                           vehicleState
                                                           pidControllers
                                                           []
    in ((mixer newDemands), newPidControllers)
