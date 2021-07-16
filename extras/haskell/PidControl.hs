{--
  General PID control

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module PidControl where

import State(Time, VehicleState)
import Demands

type PidControllerState = [Double]

type PidControllerFun = Time -> VehicleState -> Demands -> PidControllerState -> (Demands, PidControllerState)
