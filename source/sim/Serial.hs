{--
  Hackflight serial comms, stubbed for simulation

  Copyright(C) 2021 on D.Levy

  MIT License
--}

module Serial where

import Language.Copilot

import State
import Demands

data SerialGuard = SerialGuard { available :: Stream Bool, value :: Stream Word8 }

getSerialOut :: State -> Demands -> SerialGuard

getSerialOut _vehicleState _demands = SerialGuard false 0
