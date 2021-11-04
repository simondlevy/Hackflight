{--
  Control demands

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module Demands where

import Language.Copilot

import Utils

data Demands = Demands { throttle :: SFloat
                       , roll :: SFloat  
                       , pitch :: SFloat  
                       , yaw :: SFloat  
                     } deriving (Show)


addDemands :: Demands -> Demands -> Demands

addDemands d1 d2 = Demands ((throttle d1) + (throttle d2))
                           ((roll d1) + (roll d2))
                           ((pitch d1) + (pitch d2))
                           ((yaw d1) + (yaw d2))
