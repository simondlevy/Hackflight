{--
  PID controller for roll/pitch angle

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module LevelPid(levelController)

where

import Language.Copilot

import VehicleState(phi, theta)
import PidController
import Demands
import Utils(deg2rad)

levelController :: Stream Float -> Stream Float -> PidController
levelController kp maxAngleDegrees = makePidController (levelFun kp maxAngleDegrees)

levelFun :: Stream Float -> Stream Float -> PidFun
levelFun kp maxAngleDegrees vehicleState demands =

    -- Return updated demands and controller state
    Demands 0 rollDemand pitchDemand 0

    where 

      -- Maximum roll pitch demand is +/-0.5, so to convert demand to
      -- angle for error computation, we multiply by the following amount:
      demandScale = 2 * (deg2rad maxAngleDegrees)

      rollDemand = ((kp * demandScale * (roll demands)) - (phi vehicleState))

      -- Pitch demand is nose-down positive, so we negate pitch-forward
      -- vehicle state (nose-down negative) to reconcile them
      pitchDemand = ((kp * demandScale * (pitch demands)) + (theta vehicleState))
