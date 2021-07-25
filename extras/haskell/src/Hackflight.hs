{--
  Hackflight core algorithm

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module Hackflight(HackflightFun, hackflight)
where

import Mixer(Mixer, Motors)
import SimReceiver(SimReceiver, receiverDemands)
import SimSensor(SimSensor, sensorVehicleState)
import PidControl(PidController, runClosedLoop)
import VehicleState(initVehicleState)

type HackflightFun = SimReceiver ->
                     SimSensor ->
                     Mixer ->
                     [PidController] ->
                     (Motors, [PidController])

hackflight :: HackflightFun

hackflight receiver sensor mixer pidControllers =

    let 
        vehicleState = sensorVehicleState sensor

        (demands, newPidControllers) =
            runClosedLoop (receiverDemands receiver)
                          vehicleState
                          pidControllers

    in ((mixer demands), newPidControllers)
