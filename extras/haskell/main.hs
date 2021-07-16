{--
  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

import Types
import Multicopter
import AltHoldPid

main :: IO ()
main = let constants = AltHoldPidConstants 10 1 0 10
       in runMulticopter (makeAltHoldPid constants) quadXAPMixer
