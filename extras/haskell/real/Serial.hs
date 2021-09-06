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

getSerialByte :: Serial -> State -> Demands -> Stream Word8

getSerialByte _serial _vehicleState _demands = 0
