{--
  Haskell Copilot support for pass-through "sensor"

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

{-# LANGUAGE DataKinds        #-}

module SimSensor

where

import Language.Copilot

import VehicleState

simSensorZ :: Stream Double
simSensorZ = extern "simSensorZ" Nothing

simSensorDz :: Stream Double
simSensorDz = extern "simSensorDz" Nothing

simSensorModifyState :: VehicleState

simSensorModifyState =

  VehicleState 0 -- x
               0 -- dx
               0 -- y
               0 -- dy
               simSensorZ
               simSensorDz
               0 -- phi
               0 -- dphi
               0 -- theta
               0 -- dtheta
               0 -- psi
               0 -- dpsi
