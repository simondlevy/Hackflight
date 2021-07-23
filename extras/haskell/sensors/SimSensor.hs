{--
  Pass-thru sensor for testing

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module SimSensor

where

import Sensor

simSensor :: Sensor

simSensor vehicleState = vehicleState

