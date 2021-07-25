{--
  Socket-based "receiver"

  Just passes through stick demands from simulator

  Copyright(C) 2021 on D.Levy

  MIT License
--}

module SimReceiver (SimReceiver, ReceiverFun, simReceiver, receiverDemands) where

import Demands

data SimReceiver = SimReceiver { receiverDemands :: Demands } deriving (Show)

type ReceiverFun = [Double] -> SimReceiver

simReceiver :: ReceiverFun
simReceiver v =  SimReceiver $ Demands (v!!0) (v!!1) (v!!2) (v!!3) 

