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

intscale :: SWord16 -> SFloat
intscale x = ((unsafeCast x) - 512) / 512

getDemands :: Receiver -> Demands
getDemands receiver = 

    Demands throttleDemand rollDemand pitchDemand yawDemand

    where

      throttleDemand = throttleFun c_receiverThrottle

      rollDemand = ((adjustCommand (cyclicFun $ abs c_receiverRoll) c_receiverRoll) + (rollTrim trim)) * scale

      pitchDemand = ((adjustCommand (cyclicFun $ abs c_receiverPitch) c_receiverPitch) + (pitchTrim trim)) * scale

      yawDemand = ((adjustCommand  (abs c_receiverYaw)  c_receiverYaw) + (yawTrim trim)) * scale

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

c_receiverThrottle :: SFloat
c_receiverThrottle  = extern "receiverThrottle" Nothing

c_receiverRoll :: SFloat
c_receiverRoll  = extern "receiverRoll" Nothing

c_receiverPitch :: SFloat
c_receiverPitch  = extern "receiverPitch" Nothing

c_receiverYaw :: SFloat
c_receiverYaw  = extern "receiverYaw" Nothing

c_receiverReady :: SBool
c_receiverReady  = extern "receiverReady" Nothing

c_receiverTimedOut :: SBool
c_receiverTimedOut  = extern "receiverTimedOut" Nothing

c_receiverGotNewFrame :: SBool
c_receiverGotNewFrame  = extern "receiverGotNewFrame" Nothing

c_receiverAux1 :: SWord16
c_receiverAux1  = extern "receiverAux1" Nothing

c_receiverAux2 :: SWord16
c_receiverAux2  = extern "receiverAux2" Nothing
