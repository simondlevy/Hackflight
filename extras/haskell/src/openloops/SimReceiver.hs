{--
  Socket-based "receiver"

  Copyright(C) 2021 on D.Levy

  MIT License
--}

module SimReceiver (SimReceiver) where

import OpenLoopControl
import Demands

type SimReceiver = OpenLoopController

receiver :: OpenLoopController

receiver = (Demands 0 0 0 0)
