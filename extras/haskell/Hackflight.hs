{--
  Hackflight core algorithm

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module Hackflight(HackflightFun, hackflightFun)
where

-- import Sensor
import VehicleState
import Demands
import PidControl(PidController)
import Mixer(Mixer, Motors)
import ClosedLoopControl(closedLoop)

{--
type FullHackflightFun = OpenLoopController ->
                         [Sensor] ->
                         [PidController] ->
                         Mixer ->
                         (Motors, [PidController])
--}

type HackflightFun = Demands ->
                     VehicleState ->
                     Mixer ->
                     [PidController] ->
                     (Motors, [PidController])

hackflightFun :: HackflightFun

hackflightFun demands vehicleState mixer pidControllers =

    let (newDemands, newPidControllers) = closedLoop demands
                                                     vehicleState
                                                     pidControllers

    in ((mixer newDemands), newPidControllers)
