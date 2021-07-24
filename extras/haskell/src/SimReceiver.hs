{--
  Socket-based "receiver"

  Copyright(C) 2021 on D.Levy

  MIT License
--}

module SimReceiver (SimReceiver) where

import Demands

data SimReceiver = SimReceiver { receiverDemands :: Demands } deriving (Show)

simReceiver :: [Double] -> SimReceiver
simReceiver d =  SimReceiver (Demands (d!!0) (d!!1) (d!!2) (d!!3))

