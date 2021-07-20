{--
  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

import Hackflight(hackflightFun)
import Mixer(quadXAPMixer)
import Server(runServer)
import PidControl(newRateController, newAltHoldController)

main :: IO ()

main = let 
           rateController = newRateController 0.225    -- Kp
                                              0.001875 -- Ki
                                              0.375    -- Kd
                                              0.4      -- windupMax
                                              40       -- maxDegreesPerSecond

           altHoldController = newAltHoldController 0.75 -- Kp
                                                    1.5  -- Ki
                                                    0.4  -- windupMax
                                                    2.5  -- pilotVelZMax
                                                    0.2  -- stickDeadband

       in runServer hackflightFun [rateController, altHoldController] quadXAPMixer
