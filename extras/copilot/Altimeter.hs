{--
  Haskell Copilot support for simulated altimeter

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

{-# LANGUAGE DataKinds        #-}

module Altimeter

where

import Language.Copilot

import VehicleState
import Sensor

altimeterZ :: Stream Double
altimeterZ = extern "altimeterZ" Nothing

altimeterDz :: Stream Double
altimeterDz = extern "altimeterDz" Nothing

altimeter :: Sensor

altimeter vs =

  VehicleState (x vs)
               (dx vs)
               (y vs)
               (dy vs)
               ((z vs) + altimeterZ)
               ((dz vs) + altimeterDz)
               (phi vs)
               (dphi vs)
               (theta vs)
               (dtheta vs)
               (psi vs)
               (dpsi vs)
