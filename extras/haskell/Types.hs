{--
  Types for Multicopter

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module Types where

import State

type Time = Double

-------------------------------------------------------

data Demands = Demands { throttle :: Double
                       , roll :: Double  
                       , pitch :: Double  
                       , yaw :: Double  
                     } deriving (Show)

-------------------------------------------------------

-- XXX should support different numbers of motors
data Motors = Motors { m1 :: Double
                     , m2 :: Double  
                     , m3 :: Double  
                     , m4 :: Double  
                     } deriving (Show)

-------------------------------------------------------

type Mixer = Demands -> Motors

quadXAPMixer :: Mixer
quadXAPMixer demands = 
    let t = (throttle demands)
        r = (roll demands)
        p = (pitch demands)
        y = (yaw demands)
    in Motors (t - r - p - y)
              (t + r + p - y)
              (t + r - p + y)
              (t - r + p + y)
     
-------------------------------------------------------

data PidControllerState = PidControllerState { previousTime :: Double 
                                             , errorIntegral :: Double
                                             } deriving (Show)

type PidController = Time -> VehicleState -> Demands -> PidControllerState -> (Demands, PidControllerState)
