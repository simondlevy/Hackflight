{--
  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

import Hackflight(hackflightFun)
import Mixer(quadXAPMixer)
import Server(runServer)
import PidControl(newAltHoldController)

main :: IO ()

main = let altHoldController = newAltHoldController 0.75 1.5 0.4 2.5 0.2
       in runServer hackflightFun altHoldController quadXAPMixer
