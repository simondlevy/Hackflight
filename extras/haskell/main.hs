{--
  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

import Hackflight(hackflightFun)
import Mixer(quadXAPMixer)
import Server(runServer)
import PidControl(newAltHoldController)

main :: IO ()

main = let altHoldController = newAltHoldController 0.75 -- Kp
                                                    1.5  -- Ki
                                                    0.4  -- windupMax
                                                    2.5  -- pilotVelZMax
                                                    0.2  -- stickDeadband
       in runServer hackflightFun [altHoldController] quadXAPMixer
