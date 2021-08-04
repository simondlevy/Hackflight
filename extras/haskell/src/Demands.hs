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
