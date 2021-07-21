{--
  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

import Hackflight(hackflightFun)
import Mixer(quadXAPMixer)
import Server(runServer)
import PidControl(rateController,
                  levelController,
                  altHoldController,
                  yawController)

main :: IO ()

main = let 
           rate = rateController 0.225    -- Kp
                                 0.001875 -- Ki
                                 0.375    -- Kd
                                 0.4      -- windupMax
                                 40       -- maxDegreesPerSecond

           yaw = yawController 2.0 -- Kp
                               0.1 -- Ki
                               0.4 -- windupMax

           altHold = altHoldController 0.75 -- Kp
                                       1.5  -- Ki
                                       0.4  -- windupMax
                                       2.5  -- pilotVelZMax
                                       0.2  -- stickDeadband

       in runServer hackflightFun [rate, altHold, yaw] quadXAPMixer
