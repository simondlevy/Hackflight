{--
  PID control

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module PidControl where

import State
import Demands

data PidControllerState = PidControllerState { previousTime :: Double 
                                             , errorIntegral :: Double
                                             } deriving (Show)

type PidController = Time -> VehicleState -> Demands -> PidControllerState -> (Demands, PidControllerState)
