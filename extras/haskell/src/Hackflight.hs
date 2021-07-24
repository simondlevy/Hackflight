{--
  Hackflight core algorithm

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module Hackflight(HackflightFun, hackflight)
where

import VehicleState
import Mixer(Mixer, Motors)
import Receiver
import Sensor
import ClosedLoopControl(PidController, runClosedLoop)

type HackflightFun = Receiver ->
                     [Sensor] ->
                     VehicleState ->
                     Mixer ->
                     [PidController] ->
                     (Motors, [PidController])

hackflight :: HackflightFun

hackflight receiver _sensors vehicleState mixer pidControllers =

    let (newDemands, newPidControllers) = runClosedLoop receiver
                                                        vehicleState
                                                        pidControllers

    in ((mixer newDemands), newPidControllers)
