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
import OpenLoopControl(OpenLoopController)

type HackflightFun = OpenLoopController ->
                     [Sensor] ->
                     Mixer ->
                     [PidController] ->
                     (Motors, [PidController])

hackflight :: HackflightFun

hackflight receiver sensors mixer pidControllers =

    let vehicleState = initVehicleState

        demands = Demands 0 0 0 0

    in ((mixer demands), pidControllers)
