{--
  PID controller for roll/pitch angle

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module LevelPid(levelController)

where

import Language.Copilot

import State(phi, theta)
import PidController
import Demands
import Utils

angleMax = 45 :: SFloat

levelController :: SFloat -> PidFun

levelController kp (state, demands) = (state, demands')

    where

      demands' = Demands (throttle demands) rollDemand pitchDemand (yaw demands)

      rollDemand = ((kp * demandScale * (roll demands)) - (phi state))

      -- Pitch demand is nose-down positive, so we negate pitch-forward
      -- vehicle state (nose-down negative) to reconcile them
      pitchDemand = ((kp * demandScale * (pitch demands)) + (theta state))

      -- angle for error computation, we multiply by the following amount:
      demandScale = 2 * (deg2rad angleMax)
