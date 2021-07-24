{--
  Socket-based "receiver"

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module SimReceiver (simReceiver) where

import OpenLoopControl
import Demands

type SimReceiver = OpenLoopController

simReceiver :: OpenLoopController

simReceiver = (Demands 0 0 0 0)
