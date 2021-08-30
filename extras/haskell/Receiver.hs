{--
  Haskell Copilot support for RC receivers

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module Receiver where

import Language.Copilot
import Copilot.Compile.C99

import Demands

data AxisTrim = AxisTrim {  rollTrim :: Stream Float
                          , pitchTrim :: Stream Float
                          , yawTrim :: Stream Float
                         } deriving (Show)

data ChannelMap = ChannelMap {  throttleChannel :: Stream Float
                              , rollChannel :: Stream Float
                              , pitchChannel :: Stream Float
                              , yawChannel :: Stream Float
                              , aux1Channel :: Stream Float
                              , aux2Channel :: Stream Float
                             } deriving (Show)

data Receiver = Receiver {  throttleMargin :: Stream Float
                          , throttleExpo :: Stream Float
                          , cyclicExpo :: Stream Float
                          , cyclicRate :: Stream Float
                          , auxTheshold :: Stream Float
                          , demandScale :: Stream Float
                          , channelMap :: ChannelMap
                          , axisTrim :: AxisTrim
                         } deriving (Show)

makeReceiverWithTrim :: ChannelMap -> AxisTrim -> Stream Float -> Receiver
makeReceiverWithTrim channelMap axisTrim demandScale =
    Receiver 0.10 -- throttleMargin
             0.20 -- throttleExpo
             0.90 -- cyclicRate
             0.65 -- cyclicExpo
             0.40 -- auxThreshold
             demandScale
             channelMap
             axisTrim

makeReceiver :: ChannelMap -> Stream Float -> Receiver
makeReceiver channelMap demandScale =
  makeReceiverWithTrim channelMap (AxisTrim 0 0 0) demandScale

getDemands :: Receiver -> Demands
getDemands receiver = 

    Demands throttleDemand rollDemand pitchDemand yawDemand

    where

      channelMap' = channelMap receiver

      demandScale' = demandScale receiver

      axisTrim' = axisTrim receiver

      adjustCommand command channelSelector = 
          command/2 * if (channelSelector channelMap') < 0 then -1 else 1

      applyCyclicFunction command = rcFun command (cyclicExpo receiver) (cyclicRate receiver)

      makePositiveCommand channelSelector = abs (channelSelector channelMap')

      rcFun x e r = (1 + e * (x*x -1)) * x * r

      throttleFun x = 
          let mid = 0.5
              tmp = (x + 1) / 2 - mid
              y = if tmp > 0 then 1 - mid else (if tmp < 0 then mid else 1)
              expo = (throttleExpo receiver)
          in (mid + tmp*(1-expo + expo * (tmp*tmp) / (y*y))) * 2 - 1

      throttleDemand = throttleFun $ throttleChannel channelMap'

      rollDemand = demandScale' *
                   ((rollTrim axisTrim') + 
                   adjustCommand (applyCyclicFunction $ makePositiveCommand rollChannel) rollChannel)
       
      pitchDemand = demandScale' *
                   ((pitchTrim axisTrim') + 
                   adjustCommand (applyCyclicFunction $ makePositiveCommand pitchChannel) pitchChannel)
 
      yawDemand = demandScale' *
                   ((yawTrim axisTrim') + 
                   adjustCommand (makePositiveCommand yawChannel) yawChannel)

receiverReady ::  Stream Bool
receiverReady = receiverGotNewFrame

-- Externals -------------------------------------------------

chan1 :: Stream Float
chan1  = extern "copilot_receiverChannel1" Nothing

chan2 :: Stream Float
chan2  = extern "copilot_receiverChannel2" Nothing

chan3 :: Stream Float
chan3  = extern "copilot_receiverChannel3" Nothing

chan4 :: Stream Float
chan4  = extern "copilot_receiverChannel4" Nothing

chan5 :: Stream Float
chan5  = extern "copilot_receiverChannel5" Nothing

chan6 :: Stream Float
chan6  = extern "copilot_receiverChannel6" Nothing

chan7 :: Stream Float
chan7  = extern "copilot_receiverChannel7" Nothing

chan8 :: Stream Float
chan8  = extern "copilot_receiverChannel8" Nothing

receiverLostSignal :: Stream Bool
receiverLostSignal  = extern "copilot_receiverLostSignal" Nothing

receiverGotNewFrame :: Stream Bool
receiverGotNewFrame  = extern "copilot_receiverGotNewFrame" Nothing
