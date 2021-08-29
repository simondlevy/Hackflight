{--
  Haskell Copilot support for RC receivers

  Copyright(C) 2021 on D.Levy

  MIT License
--}


module Receiver where

import Language.Copilot
import Copilot.Compile.C99

-- Externals --------------------------------------------------

receiverThrottle :: Stream Float
receiverThrottle  = extern "copilot_receiverThrottle" Nothing

receiverRoll :: Stream Float
receiverRoll  = extern "copilot_receiverRoll" Nothing

receiverPitch :: Stream Float
receiverPitch  = extern "copilot_receiverPitch" Nothing

receiverYaw :: Stream Float
receiverYaw  = extern "copilot_receiverYaw" Nothing

receiverLostSignal :: Stream Bool
receiverLostSignal  = extern "copilot_receiverLostSignal" Nothing

-- Internals -------------------------------------------------

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

makeReceiverWithTrim channelMap' axisTrim' demandScale' =
    Receiver 0.10 -- throttleMargin
             0.20 -- throttleExpo
             0.90 -- cyclicRate
             0.65 -- cyclicExpo
             0.40 -- auxThreshold
             demandScale'
             channelMap'
             axisTrim'

makeReceiver :: ChannelMap -> Stream Float -> Receiver

makeReceiver channelMap' demandScale' =
  makeReceiverWithTrim channelMap' (AxisTrim 0 0 0) demandScale' 
