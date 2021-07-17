{--
  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

import Mixer
import Server
import PidControl

main :: IO ()

main = let altHoldController = newAltHoldController 10 1 0 1
       in run altHoldController quadXAPMixer
