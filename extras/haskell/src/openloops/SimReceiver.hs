{--
  Socket-based "receiver"

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module SimReceiver (simReceiver) where

import OpenLoopControl
import Demands

simReceiver :: OpenLoopController

simReceiver demands = 

  (Demands 0 0 0 0)
