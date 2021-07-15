{--
  Main Hackflight algorithm

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module Hackflight where

import Board
import Receiver

data Hackflight = Hackflight {
                   board :: Board,
                   olc :: Receiver
                  } deriving (Show)

run :: Hackflight -> Hackflight

run h = h

