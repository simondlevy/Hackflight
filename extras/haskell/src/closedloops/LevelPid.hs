{--
  PID controller for roll/pitch angle

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module LevelPid(levelController)

where

import VehicleState
import ClosedLoopControl
import Demands
import Utils(deg2rad)

levelController :: Double -> Double -> PidController

levelController kp maxAngleDegrees =
    PidController (levelClosure kp maxAngleDegrees) NoState

levelClosure :: Double -> Double -> PidFun
levelClosure kp maxAngleDegrees =

    \vehicleState -> \demands -> \_controllerState ->

    let 
        -- Maximum roll pitch demand is +/-0.5, so to convert demand to
        -- angle for error computation, we multiply by the folling amount:
        demandScale = 2 * deg2rad(maxAngleDegrees)

        rollDemand = ((kp * demandScale * (Demands.roll demands)) -
                      (VehicleState.phi vehicleState))

        -- Pitch demand is nose-down positive, so we negate
        -- pitch-forward (nose-down negative) to reconcile them
        pitchDemand = ((kp * demandScale * (Demands.pitch demands)) +
                       (VehicleState.theta vehicleState))

    -- Return updated demands and controller state
    in ((Demands (Demands.throttle demands)
                 rollDemand
                 pitchDemand
                 (Demands.yaw demands)),
        NoState)
