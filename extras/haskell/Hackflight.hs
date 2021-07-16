{--
  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

import Mixer
import Server
import AltHoldPid

main :: IO ()
main = let altitudeTarget = 10
           altKp = 1
           altKi = 0
           altWindupMax = 1
       in run (altHoldClosure altitudeTarget altKp altKi altWindupMax) newAltHoldState 
              quadXAPMixer
