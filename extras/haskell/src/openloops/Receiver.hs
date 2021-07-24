{--
  Socket-based "receiver"

  Copyright(C) 2021 on D.Levy

  MIT License
--}

module Receiver (Receiver) where

import OpenLoopControl
import Demands

type Receiver = OpenLoopController

receiver :: OpenLoopController

receiver = (Demands 0 0 0 0)
