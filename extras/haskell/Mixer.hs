{--
  Mixer type

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module Mixer where

data MotorDirections = MotorDirections {
                         d1 :: Int,
                         d2 :: Int,
                         d3 :: Int,
                         d4 :: Int } deriving (Show)

data Mixer = Op (Double->Double->Double) String (Double->Double->Double) 

instance Show Mixer where
   show (Op op str inv) = show str

-- quadXAPMixer :: Mixer
--quadXAPMixer motorDirections = \a -> \b -> a + b
 
{--
import Demands

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
--}
