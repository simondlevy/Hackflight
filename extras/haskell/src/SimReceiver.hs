{--
  Socket-based "receiver"

  Copyright(C) 2021 on D.Levy

  MIT License
--}

module SimReceiver (SimReceiver) where

import Demands

data SimReceiver = SimReceiver { receiverDemands :: Demands } deriving (Show)

--receiver :: Demands

--receiver = (Demands 0 0 0 0)
