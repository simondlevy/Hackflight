{--
  Haskell Copilot support for RC receivers

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module Receiver where

import Prelude hiding(abs, (>), (<))
import Language.Copilot
import Copilot.Compile.C99

import Demands
import Utils

data AxisTrim = AxisTrim {  rollTrim :: SFloat
                          , pitchTrim :: SFloat
                          , yawTrim :: SFloat
                         } deriving (Show)

data Receiver = Receiver {  throttleMargin :: SFloat
                          , throttleExpo :: SFloat
                          , cyclicExpo :: SFloat
                          , cyclicRate :: SFloat
                          , auxTheshold :: SFloat
                          , demandScale :: SFloat
                          , axisTrim :: AxisTrim
                         } deriving (Show)

makeReceiverWithTrim :: AxisTrim -> SFloat -> Receiver
makeReceiverWithTrim axisTrim demandScale =
    Receiver 0.10 -- throttleMargin
             0.20 -- throttleExpo
             0.65 -- cyclicExpo
             0.90 -- cyclicRate
             0.40 -- auxThreshold
             demandScale
             axisTrim

makeReceiver :: SFloat -> Receiver
makeReceiver demandScale =
  makeReceiverWithTrim (AxisTrim 0 0 0) demandScale

getDemands :: Receiver -> Demands
getDemands receiver = 

    Demands throttleDemand rollDemand pitchDemand yawDemand

    where

      throttleDemand = throttleFun receiverThrottle

      rollDemand = ((adjustCommand (cyclicFun $ abs receiverRoll) receiverRoll) + (rollTrim trim)) * scale

      pitchDemand = ((adjustCommand (cyclicFun $ abs receiverPitch) receiverPitch) + (pitchTrim trim)) * scale

      yawDemand = ((adjustCommand  (abs receiverYaw)  receiverYaw) + (yawTrim trim)) * scale

      cyclicFun command = rcFun command (cyclicExpo receiver) (cyclicRate receiver)
 
      rcFun x e r = (1 + e*(x*x - 1)) * x * r

      adjustCommand demand rawval = if rawval < 0 then -demand/2 else demand/2

      throttleFun x = 
          let mid = 0.5
              tmp = (x + 1) / 2 - mid
              y = if tmp > 0 then 1 - mid else (if tmp < 0 then mid else 1)
              expo = (throttleExpo receiver)
          in (mid + tmp*(1-expo + expo * (tmp*tmp) / (y*y))) * 2 - 1

      trim = axisTrim receiver

      scale = demandScale receiver

-- Externals -------------------------------------------------

receiverThrottle :: SFloat
receiverThrottle  = extern "stream_receiverThrottle" Nothing

receiverRoll :: SFloat
receiverRoll  = extern "stream_receiverRoll" Nothing

receiverPitch :: SFloat
receiverPitch  = extern "stream_receiverPitch" Nothing

receiverYaw :: SFloat
receiverYaw  = extern "stream_receiverYaw" Nothing

receiverAux1 :: SFloat
receiverAux1  = extern "stream_receiverAux1" Nothing

receiverAux2 :: SFloat
receiverAux2  = extern "stream_receiverAux2" Nothing

receiverReady :: SBool
receiverReady  = extern "stream_receiverReady" Nothing

receiverTimedOut :: SBool
receiverTimedOut  = extern "stream_receiverTimedOut" Nothing

receiverGotNewFrame :: SBool
receiverGotNewFrame  = extern "stream_receiverGotNewFrame" Nothing
