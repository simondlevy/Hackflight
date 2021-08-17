{--
  Vehicle state

  See Bouabdallah et al. (2004)

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module VehicleState where

import Language.Copilot

data VehicleState = VehicleState {   z :: Stream Double
                                  , dz :: Stream Double
                                 } deriving (Show)

initialVehicleState :: VehicleState

initialVehicleState = VehicleState 0 0


addStates :: VehicleState -> VehicleState -> VehicleState

addStates v1 v2 = 
    VehicleState ((z v1) + (z v2))
                 ((dz v1) + (dz v2))
