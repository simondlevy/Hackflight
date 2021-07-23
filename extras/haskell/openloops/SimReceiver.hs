{--
  "Receiver" that gets its data from simulator

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module SimReceiver

where

import Demands
import OpenLoopControl

simReceiver :: OpenLoopController

simReceiver = Demands 0 0 0 0
