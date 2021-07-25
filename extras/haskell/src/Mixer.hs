{--
  Mixer type

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module Mixer(Mixer, quadXAPMixer, Motors, motorValues) where

import Demands
import Utils(constrain)

-- XXX should support different numbers of motors
data Motors = Motors { m1 :: Double
                     , m2 :: Double  
                     , m3 :: Double  
                     , m4 :: Double  
                     } deriving (Show)

motorValues :: Motors -> [Double]
motorValues motors = [m1 motors, m2 motors, m3 motors, m4 motors]

type Mixer = Demands -> Motors

quadXAPMixer :: Mixer
quadXAPMixer demands = 
    let t = (throttle demands)
        r = (roll demands)
        p = (pitch demands)
        y = (yaw demands)
    in Motors (constrain (t - r - p + y))
              (constrain (t + r + p + y))
              (constrain (t + r - p - y))
              (constrain (t - r + p - y))
