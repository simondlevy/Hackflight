{--
  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

import Mixer
import Server(runServer)
import PidControl

main :: IO ()

main = let altHoldController = newAltHoldController 10 1 0 1
       in runServer altHoldController quadXAPMixer
