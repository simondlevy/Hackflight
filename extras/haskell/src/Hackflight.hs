{--
  Hackflight core algorithm

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module Hackflight(HackflightFun, hackflight)
where

import VehicleState
import Mixer(Mixer, Motors)
import SimReceiver(SimReceiver, receiverDemands)
import Sensor
import PidControl(PidController, runClosedLoop)

type HackflightFun = SimReceiver ->
                     [Sensor] ->
                     VehicleState ->
                     Mixer ->
                     [PidController] ->
                     (Motors, [PidController])

hackflight :: HackflightFun

hackflight receiver sensors vehicleState mixer pidControllers =

    let demands = receiverDemands receiver

        (newDemands, newPidControllers) = runClosedLoop demands
                                                        vehicleState
                                                        pidControllers

    in ((mixer newDemands), newPidControllers)
