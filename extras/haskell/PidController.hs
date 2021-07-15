{--
  PID controller type for Multicopter

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module PidController where

import State
import Types

type Time = Double

data PidControllerState = PidControllerState { previousTime :: Time 
                                             , errorIntegral :: Double
                                             } deriving (Show)

type PidController = Time -> VehicleState -> Demands -> PidControllerState -> (Demands, PidControllerState)
