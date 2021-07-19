{--
  Socket-based multicopter control

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module Tmp

where

import State
import PidControl(PidFun, PidState)

loop :: (PidFun, PidState) -> (PidFun, PidState)
loop pidController  =

      let t = 0

          s = VehicleState 0 0 0 0 0 0 0 0 0 0 0 0

          controllerFun = fst pidController

          (d, newControllerState) = controllerFun t s d (snd pidController)

      in loop (controllerFun, newControllerState)
