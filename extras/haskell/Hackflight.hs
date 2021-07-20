{--
  Hackflight core algorithm

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module Hackflight(HackflightFun, hackflightFun)
where

import State(VehicleState)
import Demands
import PidControl(PidController, newPidController, pidFun, pidState)
import Mixer(Mixer, Motors)

type HackflightFun = Demands ->
                     VehicleState ->
                     Mixer ->
                     [PidController] ->
                     (Motors, [PidController])

hackflightFun :: HackflightFun

helperFun :: Demands ->
             VehicleState ->
             Mixer ->
             [PidController] ->
             [PidController] ->
             (Motors, [PidController])

-- Base case: apply mixer to final demands to get motors
helperFun demands _ mixer [] newPidControllers =
    (mixer demands, newPidControllers)

-- Recursive case: apply current PID controller to demands to get new demands
-- and PID state; then recur on remaining PID controllers
helperFun demands vehicleState mixer oldPidControllers newPidControllers =

    let oldPidController = head oldPidControllers
   
        pfun = pidFun oldPidController

        pstate = pidState oldPidController

        (newDemands, newPstate) = pfun vehicleState demands pstate

        newPid = newPidController pfun newPstate

    in helperFun newDemands
                     vehicleState
                     mixer (tail oldPidControllers)
                     (newPid:newPidControllers)

hackflightFun demands vehicleState mixer pidControllers =
    helperFun demands vehicleState mixer pidControllers []
