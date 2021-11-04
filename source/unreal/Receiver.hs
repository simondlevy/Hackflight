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
             0.90 -- cyclicRate
             0.65 -- cyclicExpo
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

      demandScale' = demandScale receiver

      axisTrim' = axisTrim receiver

      adjustCommand rectified original =
          rectified/2 * if original < 0 then -1 else 1

      cyclicFun command = rcFun command (cyclicExpo receiver) (cyclicRate receiver)

      rectify x = if x < 0 then (-x) else x

      rcFun x e r = (1 + e * (x*x -1)) * x * r

      angleFun trimFun expoFun command =
          demandScale' * ((trimFun axisTrim') + adjustCommand (expoFun $ rectify command)
                                                              command)

      throttleFun x = 
          let mid = 0.5
              tmp = (x + 1) / 2 - mid
              y = if tmp > 0 then 1 - mid else (if tmp < 0 then mid else 1)
              expo = (throttleExpo receiver)
          in (mid + tmp*(1-expo + expo * (tmp*tmp) / (y*y))) * 2 - 1

      throttleDemand = throttleFun receiverThrottle

      rollDemand = angleFun rollTrim cyclicFun receiverRoll

      pitchDemand = angleFun pitchTrim cyclicFun receiverPitch

      yawDemand = angleFun yawTrim id receiverYaw

-- a.k.a. inactive
receiverThrottleIsDown :: SBool
receiverThrottleIsDown = receiverThrottle < (-0.995)

-- Externals -------------------------------------------------

receiverThrottle :: SFloat
receiverThrottle  = extern "copilot_receiverThrottle" Nothing

receiverRoll :: SFloat
receiverRoll  = extern "copilot_receiverRoll" Nothing

receiverPitch :: SFloat
receiverPitch  = extern "copilot_receiverPitch" Nothing

receiverYaw :: SFloat
receiverYaw  = extern "copilot_receiverYaw" Nothing

receiverAux1 :: SFloat
receiverAux1  = extern "copilot_receiverAux1" Nothing

receiverAux2 :: SFloat
receiverAux2  = extern "copilot_receiverAux2" Nothing

receiverLostSignal :: SBool
receiverLostSignal  = extern "copilot_receiverLostSignal" Nothing
