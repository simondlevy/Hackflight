{--
  Sensor support

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module Sensor

where

import VehicleState

-- type Sensor = IO () -> VehicleState -> VehicleState

type Sensor = VehicleState -> VehicleState
