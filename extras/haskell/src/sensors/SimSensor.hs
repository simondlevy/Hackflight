{--
  Socket-based "sensor"

  Just passes through state telemetry from simulator

  Copyright(C) 2021 on D.Levy

  MIT License
--}

module SimSensor (simSensorClosure) where

import VehicleState
import Sensor

simSensorClosure :: [Double] -> Sensor

simSensorClosure v =

    -- Ignore input and return state made from socket telemetry values
    \_vehicleState -> VehicleState (v!!0) (v!!1) (v!!2) (v!!3) (v!!4) (v!!5)
                                   (v!!6) (v!!7) (v!!8) (v!!9) (v!!10) (v!!11)
