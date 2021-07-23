{--
  Hackflight core algorithm

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module Hackflight(HackflightFun, hackflight)
where

import Sensor(Sensor, runSensors)
import VehicleState
import Demands
import Mixer(Mixer, Motors)
import ClosedLoopControl(PidController, runClosedLoop)

type HackflightFun = Demands ->
                     [Sensor] ->
                     VehicleState ->
                     Mixer ->
                     [PidController] ->
                     (Motors, [PidController])

hackflight :: HackflightFun

hackflight demands sensors vehicleState mixer pidControllers =

    let newVehicleState = runSensors sensors vehicleState

        (newDemands, newPidControllers) = runClosedLoop demands
                                                        vehicleState
                                                        pidControllers

    in ((mixer newDemands), newPidControllers)
