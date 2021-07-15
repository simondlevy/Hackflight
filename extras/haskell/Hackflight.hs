{--
  Main Hackflight algorithm

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module Hackflight where

import Board

data Hackflight = Hackflight {board :: Board } deriving (Show)

run :: Hackflight -> Hackflight

run h = h

