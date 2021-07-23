{--
  Hackflight core algorithm

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module Hackflight(HackflightFun, hackflight, HackflightFun2, hackflight2)
where

import Sensor(Sensor, runSensors)
import VehicleState
import Demands
import Mixer(Mixer, Motors)
import ClosedLoopControl(PidController, runClosedLoop)
import OpenLoopControl(OpenLoopController)

type HackflightFun2 = OpenLoopController ->
                      [Sensor] ->
                      Mixer ->
                      [PidController] ->
                      (Motors, [PidController])

hackflight2 :: HackflightFun2

hackflight2 receiver sensors mixer pidControllers =

    let vehicleState = initVehicleState

        demands = Demands 0 0 0 0

    in ((mixer demands), pidControllers)

-------------------------------------------------------------------------

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
