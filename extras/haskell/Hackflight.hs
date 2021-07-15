{--
  Main Hackflight algorithm

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module Hackflight where

import Board
import Receiver
import Mixer

data Hackflight = Hackflight {
                   board :: Board,
                   olc :: Receiver,
                   mixer :: Mixer
                  } deriving (Show)

run :: Hackflight -> Hackflight

run h = h

