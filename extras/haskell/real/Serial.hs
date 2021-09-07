{--
  Hackflight serial comms

  Copyright(C) 2021 on D.Levy

  MIT License
--}

module Serial where

import Language.Copilot

import State
import Demands

data SerialGuard = SerialGuard { available :: Stream Bool, value :: Stream Word8 }

getSerialOut :: SerialGuard -> State -> Demands -> SerialGuard

getSerialOut _serialIn _vehicleState _demands = SerialGuard false 0


serialByteIn :: Byte
serialByteIn = extern "copilot_serialByte" Nothing
