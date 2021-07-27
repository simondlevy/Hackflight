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
                          []

    in ((getMotors mixer demands), newPidControllers)

--------------------------------------------------------------------------------

pidUpdate :: VehicleState -> (Demands, PidController) -> 
             (Demands, PidController)

pidUpdate vehicleState (demands, pidController) = 

    let pfun = pidFun pidController

        (newDemands, newPstate) = pfun vehicleState
                                  demands
                                  (pidState pidController)

        newPid = newPidController pfun newPstate

    in (newDemands, newPid)

-- Runs PID (closed-loop) controllers on vehicle state and demands to get 
-- new demands and controller states
runClosedLoop :: Demands -> VehicleState -> [PidController] -> [PidController]
                  -> (Demands, [PidController])


-- Base case: ignore vehicle state and return new demands and new PID
-- controllers
runClosedLoop newDemands _vehicleState [] newPidControllers =
    (newDemands, newPidControllers)


-- Recursive case: apply current PID controller to demands to get new
-- demands and PID state; then recur on remaining PID controllers
runClosedLoop oldDemands vehicleState oldPidControllers newPidControllers =

    let (newDemands, newPid) = pidUpdate vehicleState
                                         (oldDemands, (head oldPidControllers))

    in runClosedLoop  newDemands
                      vehicleState
                      (tail oldPidControllers)
                      (newPidControllers ++ [newPid])
