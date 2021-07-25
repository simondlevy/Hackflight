{--
  Socket-based "sensor"

  Just passes through state telemetry from simulator

  Copyright(C) 2021 on D.Levy

  MIT License
--}

module SimSensor (SimSensor, SensorFun, simSensor, sensorVehicleState) where

import VehicleState
import Sensor

{--
simSensorClosure :: Double ->
                    Double ->
                    Double ->
                    Double ->
                    Double ->
                    Double ->
                    Double ->
                    Double ->
                    Double ->
                    Double ->
                    Double ->
                    Double ->
                    Sensor

-- Ignore input and return a fixed state
simSensorClosure x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 x11 x12 =
    \vehicleState -> VehicleState x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 x11 x12 
--}
 
data SimSensor = SimSensor { sensorVehicleState :: VehicleState } deriving (Show)

type SensorFun = [Double] -> SimSensor

simSensor :: SensorFun
simSensor v =
    SimSensor $ VehicleState (v!!0) (v!!1) (v!!2) (v!!3) (v!!4) (v!!5)
                             (v!!6) (v!!7) (v!!8) (v!!9) (v!!10) (v!!11)

