{--
  Haskell Copilot support for simulated gyrometer

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

{-# LANGUAGE DataKinds        #-}

module SimGyrometer

where

import Language.Copilot

import VehicleState

simGyroX :: Stream Double
simGyroX = extern "simGyroX" Nothing

simGyroY :: Stream Double
simGyroY = extern "simGyroY" Nothing

simGyroZ :: Stream Double
simGyroZ = extern "simGyroZ" Nothing

simGyroModifyState :: VehicleState

simGyroModifyState =

  VehicleState 0 -- x
               0 -- dx
               0 -- y
               0 -- dy
               0 -- z
               0 -- dz
               0 -- phi
               simGyroX 
               0 -- theta
               simGyroY
               0 -- psi
               simGyroZ 
