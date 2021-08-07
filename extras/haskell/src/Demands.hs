{--
  Control demands

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module Demands where

data Demands = Demands { throttle :: Double
                       , roll :: Double  
                       , pitch :: Double  
                       , yaw :: Double  
                     } deriving (Show)

initialDemands :: Demands

initialDemands = Demands 0 0 0 0


addDemands :: Demands -> Demands -> Demands

addDemands d1 d2 = Demands ((throttle d1) + (throttle d2))
                           ((roll d1) + (roll d2))
                           ((pitch d1) + (pitch d2))
                           ((yaw d1) + (yaw d2))
  

