{--
  Hackflight core algorithm

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module Hackflight(HackflightFun, hackflight)
where

import VehicleState
import Demands
import Mixer(Mixer, Motors)
import Receiver
import ClosedLoopControl(PidController, runClosedLoop)

type HackflightFun = Receiver ->
                     VehicleState ->
                     Mixer ->
                     [PidController] ->
                     (Motors, [PidController])

hackflight :: HackflightFun

hackflight receiver vehicleState mixer pidControllers =

    let (newDemands, newPidControllers) = runClosedLoop receiver
                                                        vehicleState
                                                        pidControllers

    in ((mixer newDemands), newPidControllers)
