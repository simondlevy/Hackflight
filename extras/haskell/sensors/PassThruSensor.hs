{--
  Pass-thru sensor for testing

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module PassThruSensor

where

import Sensor

passThruSensor :: Sensor

passThruSensor vehicleState = vehicleState

