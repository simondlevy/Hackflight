{--
  Sensor support

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module Sensor

where

import VehicleState

data Sensor = SimSensor Double Double Double Double Double Double
                        Double Double Double Double Double Double

modifyState :: Sensor -> VehicleState -> VehicleState

-- Simulated sensor ignores vehicle state and return sate made from
-- values received over socket
modifyState (SimSensor v1 v2 v3 v4 v5 v6 v7 v8 v9 v10 v11 v12) _ =
  VehicleState v1 v2 v3 v4 v5 v6 v7 v8 v9 v10 v11 v12
