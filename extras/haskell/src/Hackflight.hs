{--
  Hackflight core algorithm

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module Hackflight(HackflightFun, hackflight)
where

import Receiver(Receiver, getDemands)
import Sensor(Sensor, runSensors)
import Mixer(Mixer, Motors)
import PidControl(PidController, runClosedLoop)
import VehicleState(initialVehicleState)

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

    in ((mixer demands), newPidControllers)
