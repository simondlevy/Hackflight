{--
  Hackflight time support

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module Time where

import Language.Copilot

import Utils

{--
import Prelude hiding (div, (>), (==), (++))

ready :: SWord32 -> SBool
  
ready freq = (micros == micros_prev) where

  period = div 1000000 freq
  micros_prev = if (micros - micros_prev') > period then micros else micros_prev'
  micros_prev' = [0] ++ micros_prev
--}

micros :: SWord32
micros  = extern "copilot_micros" Nothing

time :: SFloat
time  = extern "copilot_time" Nothing
