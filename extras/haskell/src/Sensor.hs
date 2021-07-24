{--
  Sensor support

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module Sensor

where

import VehicleState

type Sensor = VehicleState ->  VehicleState

runSensors :: [Sensor] -> VehicleState -> VehicleState

runSensors [] vehicleState = vehicleState

runSensors sensors vehicleState = 

    runSensors (tail sensors) ((head sensors) vehicleState)
