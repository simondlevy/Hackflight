{--
  Socket-based "receiver"

  Just passes through stick demands from simulator

  Copyright(C) 2021 on D.Levy

  MIT License
--}

module SimReceiver (SimReceiver, ReceiverFun, simReceiver, receiverDemands) where

import Demands

data SimReceiver = SimReceiver { receiverDemands :: Demands } deriving (Show)

type ReceiverFun = Double -> Double -> Double -> Double -> SimReceiver

simReceiver :: ReceiverFun
simReceiver t r p y =  SimReceiver $ Demands t r p y

