{--
  Hackflight time support

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module Time where

import Language.Copilot

import Prelude hiding(div, (++), (>), (==), not)

import Utils

timerReady :: SWord32 -> SBool
timerReady freq = (c_usec == usec') where

  period = div 1000000 freq

  usec' = if (c_usec - usec'') > period then c_usec else usec''

  usec'' = [0] ++ usec'

runstate :: (SBool, SBool)
runstate = (running, starting) where
  flipflop = not flipflop' where flipflop' = [False] ++ flipflop
  running = if not flipflop then true else running' where running' = [False] ++ running
  starting = not running

c_usec :: SWord32
c_usec  = extern "usec" Nothing

c_time :: SFloat
c_time  = extern "time" Nothing
