{--
  Socket-based "sensor"

  Just passes through state telemetry from simulator

  Copyright(C) 2021 on D.Levy

  MIT License
--}

module SimSensor (SimSensor, SensorFun, simSensor, sensorVehicleState) where

import VehicleState

data SimSensor = SimSensor { sensorVehicleState :: VehicleState } deriving (Show)

type SensorFun = Double ->
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
                 SimSensor

simSensor :: SensorFun
simSensor x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 x11 x12 =
    SimSensor $ VehicleState x1 x2 x3 x4 x5 x6 x7 x8 x9 x10 x11 x12 

