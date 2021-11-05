{--
  Hackflight core algorithms

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module Hackflight where

import Language.Copilot

import Receiver
import State
import Sensor
import Demands
import PidController
import Safety
import Time
import Mixers
import Parser
import Utils

motorfun :: SBool -> SFloat -> SWord8 -> SWord8 -> SWord8 -> SFloat
motorfun armed flying_value index target percent =
  if armed then flying_value
  else if index == target then (unsafeCast percent) / 100
  else 0

------------------------------------------------------------


hackflight :: Receiver -> [Sensor] -> [PidFun] -> (State -> State) -> Mixer -> (SFloat -> SFloat)
  -> (Demands, State, Demands, Motors)

hackflight receiver sensors pidfuns statefun mixer mixfun
  = (rdemands, vstate, pdemands, motors)

  where

    -- Get receiver demands from external C functions
    rdemands = getDemands receiver

    -- Get the vehicle state by composing the sensor functions over the current state
    vstate = compose sensors (statefun vstate)

    -- Periodically get the demands by composing the PID controllers over the receiver
    -- demands
    (_, _, pdemands) = compose pidfuns (vstate, timerReady 300, rdemands)

    -- Run mixer on demands to get motor values
    motors = mixer mixfun pdemands

-------------------------------------------------------------------------------

hackflightFull :: Receiver -> [Sensor] -> [PidFun] -> Mixer
  -> (State, SBool, Motors, SBool)

hackflightFull receiver sensors pidfuns mixer
  = (vstate, armed', motors, led)

  where

    -- Check safety (arming / failsafe)
    (armed, failsafe, mzero) = safety rdemands vstate

    -- Disables motors on failsafe, throttle-down, ...
    safemix = \m -> if mzero then 0 else m

    -- Run the core Hackflight algorithm
    (rdemands, vstate, pdemands, motors) = hackflight receiver sensors pidfuns state' mixer safemix

    -- Blink LED during first couple of seconds; keep it solid when armed
    led = if micros < 2000000 then (mod (div micros 50000) 2 == 0) else armed

    -- Track previous value of arming state to support shutting of motors on
    -- disarm and setting them over serial connection from GCS
    armed' = [False] ++ armed

-------------------------------------------------------------------------------

hackflightSim :: Receiver -> [Sensor] -> [PidFun] -> Mixer -> Motors

hackflightSim receiver sensors pidfuns mixer = motors

  where

    (_, _, _, motors) = hackflight receiver sensors pidfuns statefun mixer mixfun

    mixfun = \m -> constrain m

    statefun = \_ -> State 0 0 0 0 0 0 0 0 0 0 0 0
