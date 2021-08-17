{--
  Haskell Copilot support for simulated altimeter

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

{-# LANGUAGE DataKinds        #-}

module SimAltimeter

where

import Language.Copilot

import VehicleState

simAltimeterZ :: Stream Double
simAltimeterZ = extern "simAltimeterZ" Nothing

simAltimeterDz :: Stream Double
simAltimeterDz = extern "simAltimeterDz" Nothing

simAltimeterModifyState :: VehicleState

simAltimeterModifyState =

  VehicleState 0 -- x
               0 -- dx
               0 -- y
               0 -- dy
               simAltimeterZ
               simAltimeterDz
               0 -- phi
               0 -- dphi
               0 -- theta
               0 -- dtheta
               0 -- psi
               0 -- dpsi
