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
