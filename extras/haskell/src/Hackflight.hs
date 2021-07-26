{--
  Hackflight core algorithm

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module Hackflight(HackflightFun, hackflight)
where

import Demands
import Receiver(Receiver, getDemands)
import Sensor(Sensor, modifyState)
import Mixer(Mixer, getMotors)
import Motor(Motors)
import PidControl
import VehicleState(VehicleState, initialVehicleState)

type HackflightFun = Receiver ->
                     [Sensor] ->
                     [PidController] ->
                     Mixer ->
                     (Motors, [PidController])

hackflight :: HackflightFun

hackflight receiver sensors pidControllers mixer =

    let 
        (demands, newPidControllers) =
            runClosedLoop (getDemands receiver)
                          (foldr modifyState initialVehicleState sensors)
                          pidControllers

    in ((getMotors mixer demands), newPidControllers)

--------------------------------------------------------------------------------

pidUpdate :: VehicleState -> PidController -> Demands ->
             (Demands, PidController)

pidUpdate vehicleState pidController demands = 

    let pfun = pidFun pidController

        (newDemands, newPstate) = pfun vehicleState
                                  demands
                                  (pidState pidController)

        newPid = newPidController pfun newPstate

    in (newDemands, newPid)

-- Runs PID (closed-loop) controllers on vehicle state and demands to get 
-- new demands and controller states
closedLoopHelper :: VehicleState -> Demands -> [PidController]
                  -> [PidController]
                  -> (Demands, [PidController])


-- Base case: ignore vehicle state and return new demands and new PID
-- controllers
closedLoopHelper _vehicleState newDemands [] newPidControllers =
    (newDemands, newPidControllers)


-- Recursive case: apply current PID controller to demands to get new
-- demands and PID state; then recur on remaining PID controllers
closedLoopHelper vehicleState oldDemands oldPidControllers newPidControllers =

    let (newDemands, newPid) = pidUpdate vehicleState
                                         (head oldPidControllers)
                                         oldDemands

    in closedLoopHelper vehicleState
                        newDemands
                        (tail oldPidControllers)
                        (newPidControllers ++ [newPid])

runClosedLoop :: Demands ->
                 VehicleState ->
                 [PidController] ->
                 (Demands, [PidController])

runClosedLoop demands vehicleState pidControllers =

    closedLoopHelper vehicleState demands pidControllers []
