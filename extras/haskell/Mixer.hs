{--
  Mixer type

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module Mixer where

import Language.Copilot

import Demands
import Utils(constrain)

data Mixer = QuadXAPMixer 

type MixerFun = Mixer -> Demands -> Stream Float

getMotor1 :: MixerFun
getMotor1 QuadXAPMixer demands = constrain (t - r - p + y)
  where (t, r, p, y) = getDemands demands

getMotor2 :: MixerFun
getMotor2 QuadXAPMixer demands = constrain (t + r + p + y)
  where (t, r, p, y) = getDemands demands

getMotor3 :: MixerFun
getMotor3 QuadXAPMixer demands = constrain (t + r - p - y)
  where (t, r, p, y) = getDemands demands

getMotor4 :: MixerFun
getMotor4 QuadXAPMixer demands = constrain (t - r + p - y)
  where (t, r, p, y) = getDemands demands

getDemands :: Demands -> (Stream Float, Stream Float, Stream Float, Stream Float)
getDemands demands = (throttle demands, roll demands, pitch demands, yaw demands)
