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
timerReady freq = (micros == micros') where

  period = div 1000000 freq

  micros' = if (micros - micros'') > period then micros else micros''

  micros'' = [0] ++ micros'

runstate :: (SBool, SBool)
runstate = (running, starting) where
  flipflop = not flipflop' where flipflop' = [False] ++ flipflop
  running = if not flipflop then true else running' where running' = [False] ++ running
  starting = not running

micros :: SWord32
micros  = extern "stream_micros" Nothing
