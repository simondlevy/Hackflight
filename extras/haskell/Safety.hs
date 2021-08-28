{--
  Safety features (fail-safe, same-to-arm)

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module Safety where

import Language.Copilot

data Safety = Safety {   armed :: Stream Bool
                    , failsafe :: Stream Bool

              } deriving (Show)
