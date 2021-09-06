{--
  Hackflight serial comms

  Copyright(C) 2021 on D.Levy

  MIT License
--}

module Serial where

import Language.Copilot

import State
import Demands

data Serial = Serial { }

type Byte = Stream Word8

getSerialByte :: Serial -> State -> Demands -> Byte

getSerialByte _serial _vehicleState _demands = 0

serialByteIn :: Byte
serialByteIn  = extern "copilot_serialByteIn" Nothing
