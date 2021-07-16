{--
  Types for Multicopter

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module Types where

type Time = Double

-- See Bouabdallah et al. (2004)
data VehicleState = VehicleState { 
                     state_x :: Double
                   , state_dx :: Double 
                   , state_y :: Double
                   , state_dy :: Double 
                   , state_z :: Double
                   , state_dz :: Double 
                   , state_phi :: Double
                   , state_dphi :: Double 
                   , state_theta :: Double
                   , state_dtheta :: Double 
                   , state_psi :: Double
                   , state_dpsi :: Double 
                   } deriving (Show)

makeState :: [Double] -> VehicleState
makeState v = VehicleState (v!!0) (v!!1) (v!!2) (v!!3) (v!!4) (v!!5) (v!!6) (v!!7) (v!!8) (v!!9) (v!!10) (v!!11)

-------------------------------------------------------

data Demands = Demands { throttle :: Double
                       , roll :: Double  
                       , pitch :: Double  
                       , yaw :: Double  
                     } deriving (Show)

-------------------------------------------------------

-- XXX should support different numbers of motors
data Motors = Motors { m1 :: Double
                     , m2 :: Double  
                     , m3 :: Double  
                     , m4 :: Double  
                     } deriving (Show)

-------------------------------------------------------

type Mixer = Demands -> Motors

quadXAPMixer :: Mixer
quadXAPMixer demands = 
    let t = (throttle demands)
        r = (roll demands)
        p = (pitch demands)
        y = (yaw demands)
    in Motors (t - r - p - y)
              (t + r + p - y)
              (t + r - p + y)
              (t - r + p + y)
     
-------------------------------------------------------

data PidControllerState = PidControllerState { previousTime :: Double 
                                             , errorIntegral :: Double
                                             } deriving (Show)

type PidController = Time -> VehicleState -> Demands -> PidControllerState -> (Demands, PidControllerState)
