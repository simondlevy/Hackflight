{--
  Hackflight safety checking

  Copyright(C) 2021 on D.Levy

  MIT License
--}

module Safety where

import Language.Copilot

import State

data Safety = Safety { armed :: Stream Bool, failsafe :: Stream Bool }

getSafety :: Stream Bool -> State -> Safety

getSafety _ _ = Safety armed failsafe

  where

    -- Simulation is always armed and never loses signal
    armed = true
    failsafe = false
