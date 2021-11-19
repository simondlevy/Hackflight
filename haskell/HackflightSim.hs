{--
  Hackflight simulation mode

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module HackflightSim where

import Language.Copilot

import Receiver
import State
import Demands
import PidController
import Time
import Mixers
import Motors
import Dynamics
import Utils


hackflight :: Receiver
           -> WorldParams
           -> VehicleParams
           -> FixedPitchParams
           -> [PidFun]
           -> Mixer
           -> SFloat
           -> SFloat
           -> (State, Motors)

hackflight receiver wparams vparams fpparams pidfuns mixer time agl = (state, motors)

  where

    -- Get receiver demands from external C functions
    rdemands = getDemands receiver

    -- Get vehicle state directly from simulation dynamics, instead of sensors
    state = dynamics wparams vparams fpparams (motors' motors) time agl

    -- Periodically get the demands by composing the PID controllers over the previous
  -- state and the current receiver demands
    (_, _, pdemands) = compose pidfuns ((state' state), timerReady 300, rdemands)

    -- Run mixer on demands to get motor values
    motors = mix constrain pdemands mixer
