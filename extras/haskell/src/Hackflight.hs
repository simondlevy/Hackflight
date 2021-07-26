{--
  Hackflight core algorithm

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module Hackflight(HackflightFun, hackflight)
where

import Receiver(Receiver, getDemands)
import Sensor(Sensor, modifyState)
import Mixer(Mixer, getMotors)
import Motor(Motors)
import PidControl(PidController, runClosedLoop)
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
