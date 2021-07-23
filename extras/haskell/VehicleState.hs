{--
  Vehicle state

  See Bouabdallah et al. (2004)

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module VehicleState where

data VehicleState = VehicleState { 
                     x :: Double
                   , dx :: Double 
                   , y :: Double
                   , dy :: Double 
                   , z :: Double
                   , dz :: Double 
                   , phi :: Double
                   , dphi :: Double 
                   , theta :: Double
                   , dtheta :: Double 
                   , psi :: Double
                   , dpsi :: Double 
                   } deriving (Show)

makeVehicleState :: VehicleState
makeVehicleState = VehicleState 0 0 0 0 0 0 0 0 0 0 0 0

