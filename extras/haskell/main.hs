{--
  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

import Mixer
import Server
import AltitudeController

main :: IO ()
main = let constants = AltitudeControllerConstants 10 1 0 10
       in runMulticopter (makeAltitudeController constants) quadXAPMixer
