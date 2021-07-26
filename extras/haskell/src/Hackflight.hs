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
                          (runSensors sensors initialVehicleState)
                          pidControllers

    in ((getMotors mixer demands), newPidControllers)


--------------------------------------------------------------------------------

-- XXX should use fold
runSensors :: [Sensor] -> VehicleState -> VehicleState

runSensors [] vehicleState = vehicleState

runSensors sensors vehicleState = 

    runSensors (tail sensors) (modifyState (head sensors) vehicleState)

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
