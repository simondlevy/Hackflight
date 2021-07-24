{--
  Socket-based "receiver"

  Copyright(C) 2021 on D.Levy

  MIT License
--}

module SimReceiver (SimReceiver) where

import Demands

type SimReceiver = Demands

receiver :: Demands

receiver = (Demands 0 0 0 0)
