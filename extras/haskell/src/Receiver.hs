{--
  Receiver support

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module Receiver

where

import Demands

data Receiver = SimReceiver Double Double Double Double
                            
getDemands :: Receiver -> Demands

getDemands (SimReceiver t r p y) = Demands t r p y
