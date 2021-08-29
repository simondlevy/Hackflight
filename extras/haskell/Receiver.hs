{--
  Haskell Copilot support for RC receivers

  Copyright(C) 2021 on D.Levy

  MIT License
--}


module Receiver where

import Language.Copilot
import Copilot.Compile.C99

import Demands

data AxisTrim = AxisTrim {  rollTrim :: Stream Float
                          , pitchTrim :: Stream Float
                          , yawTrime :: Stream Float
                         } deriving (Show)

data ChannelMap = ChannelMap {  channelThrottle :: Stream Int8
                              , channelRoll :: Stream Int8
                              , channelPitch :: Stream Int8
                              , channelYaw :: Stream Int8
                              , channelAux1 :: Stream Int8
                              , channelAux2 :: Stream Int8
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
getDemands _receiver = Demands 0 0 0 0

receiverReady ::  Stream Bool
receiverReady = receiverGotNewFrame

receiverChannel1 :: Stream Float
receiverChannel1  = extern "copilot_receiverChannel1" Nothing

receiverChannel2 :: Stream Float
receiverChannel2  = extern "copilot_receiverChannel2" Nothing

receiverChannel3 :: Stream Float
receiverChannel3  = extern "copilot_receiverChannel3" Nothing

receiverChannel4 :: Stream Float
receiverChannel4  = extern "copilot_receiverChannel4" Nothing

receiverChannel5 :: Stream Float
receiverChannel5  = extern "copilot_receiverChannel5" Nothing

receiverChannel6 :: Stream Float
receiverChannel6  = extern "copilot_receiverChannel6" Nothing

receiverChannel7 :: Stream Float
receiverChannel7  = extern "copilot_receiverChannel7" Nothing

receiverChannel8 :: Stream Float
receiverChannel8  = extern "copilot_receiverChannel8" Nothing

receiverLostSignal :: Stream Bool
receiverLostSignal  = extern "copilot_receiverLostSignal" Nothing

receiverGotNewFrame :: Stream Bool
receiverGotNewFrame  = extern "copilot_receiverGotNewFrame" Nothing


