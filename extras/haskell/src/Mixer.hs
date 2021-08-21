{--
  Mixer type

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module Mixer where

import Demands
import Motor(Motors, Motors(QuadMotors), makeMotor)

data Mixer = QuadXAPMixer 

getMotors :: Mixer -> Demands -> Motors

getMotors QuadXAPMixer demands =

    let t = (throttle demands)
        r = (roll demands)
        p = (pitch demands)
        y = (yaw demands)

    in QuadMotors (makeMotor (t - r - p + y))
                  (makeMotor (t + r + p + y))
                  (makeMotor (t + r - p - y))
                  (makeMotor (t - r + p - y))
