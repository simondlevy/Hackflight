{--
  Hackflight core algorithm

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module Hackflight(HackflightFun, hackflightFun)
where

import Sensor
import VehicleState
import Demands
import Mixer(Mixer, Motors)
import ClosedLoopControl(PidController, closedLoop)

type HackflightFun = Demands ->
                     [Sensor] ->
                     VehicleState ->
                     Mixer ->
                     [PidController] ->
                     (Motors, [PidController])

hackflightFun :: HackflightFun

hackflightFun demands _sensors vehicleState mixer pidControllers =

    let (newDemands, newPidControllers) = closedLoop demands
                                                     vehicleState
                                                     pidControllers

    in ((mixer newDemands), newPidControllers)
