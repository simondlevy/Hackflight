{--
  Hackflight safety checking

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module Safety where

import Language.Copilot

import Receiver
import State

data Safety = Safety { armed :: Stream Bool, failsafe :: Stream Bool }

getSafety :: Stream Bool -> State -> Safety

getSafety starting vehicleState = Safety armed failsafe

  where

    failsafe = false

    armed = true
