{--
  Hackflight time support

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module Time where

import Language.Copilot
import Prelude hiding (div, (>), (==), (++))

ready :: Stream Word32 -> Stream Bool
  
ready freq = (micros == micros_prev) where

  period = div 1000000 freq
  micros_prev = if (micros - micros_prev') > period then micros else micros_prev'
  micros_prev' = [0] ++ micros_prev

micros :: Stream Word32
micros  = extern "copilot_micros" Nothing
