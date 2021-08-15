{--
  Vehicle state

  See Bouabdallah et al. (2004)

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module VehicleState where

import Language.Copilot

data VehicleState = VehicleState { 
                     x :: Stream Double
                   , dx :: Stream Double 
                   , y :: Stream Double
                   , dy :: Stream Double 
                   , z :: Stream Double
                   , dz :: Stream Double 
                   , phi :: Stream Double
                   , dphi :: Stream Double 
                   , theta :: Stream Double
                   , dtheta :: Stream Double 
                   , psi :: Stream Double
                   , dpsi :: Stream Double 
                   } deriving (Show)

initialVehicleState :: VehicleState

initialVehicleState = VehicleState 0 0 0 0 0 0 0 0 0 0 0 0

