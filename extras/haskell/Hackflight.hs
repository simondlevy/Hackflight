{--
  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

import Mixer
import Server
import AltHoldPid

main :: IO ()
main = let altitudeTarget = 10
       in run (altHoldClosure altitudeTarget 1 0 1) newAltHoldState quadXAPMixer
