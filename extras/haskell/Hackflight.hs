{--
  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

import Mixer
import Server
import AltHoldPid

main :: IO ()
main = run (altHoldPid 10 1 0 1) quadXAPMixer
