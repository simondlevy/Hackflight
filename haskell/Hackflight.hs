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
import Motors
import Parser
import Serial
import Utils

hackflight :: Receiver -> [Sensor] -> [PidFun] -> StateFun -> Mixer -> SafetyFun
  -> (Demands, State, Demands, Motors)

hackflight receiver sensors pidfuns statefun mixer safefun
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
    motors = (mixerfun mixer) safefun pdemands

-------------------------------------------------------------------------------

hackflightFull :: Receiver -> [Sensor] -> [PidFun] -> Mixer
  -> (MessageBuffer, Motors, SBool)

hackflightFull receiver sensors pidfuns mixer
  = (msgbuff, motors, led)

  where

    -- Check safety (arming / failsafe)
    (armed, failsafe, mzero) = safety rdemands vstate

    -- Disables motors on failsafe, throttle-down, ...
    safemix = \m -> if mzero then 0 else m

    -- Run the core Hackflight algorithm
    (rdemands, vstate, pdemands, motors') = hackflight receiver
                                                       sensors
                                                       pidfuns
                                                       state'
                                                       mixer
                                                       safemix

    -- Blink LED during first couple of seconds; keep it solid when armed
    -- led = if micros < 2000000 then (mod (div micros 50000) 2 == 0) else armed
    led = if micros < 4000000 then (mod (div micros 50000) 2 == 0) else armed

    -- Run the serial comms parser
    (msgtyp, sending, payindex, _checked) = parse stream_serialAvailable stream_serialByte

    -- Convert the message into a buffer to send to GCS
    msgbuff = message sending msgtyp vstate

    -- Check for incoming SET_MOTOR messages from GCS
    motor_index = if msgtyp == 215 && payindex == 1 then stream_serialByte
                  else motor_index' where motor_index' = [0] ++ motor_index
    motor_percent = if msgtyp == 215 && payindex == 2 then stream_serialByte
                    else motor_percent' where motor_percent' = [0] ++ motor_percent

    -- Set motors based on arming state and whether we have GCS input
    motors = (motorfun mixer) motors' armed motor_index motor_percent

-------------------------------------------------------------------------------
hackflightSim :: Receiver -> [Sensor] -> [PidFun] -> Mixer -> Motors

hackflightSim receiver sensors pidfuns mixer = motors

  where

    (_, _, _, motors) = hackflight receiver sensors pidfuns statefun mixer safefun

    safefun = \m -> constrain m

    statefun = \_ -> State 0 0 0 0 0 0 0 0 0 0 0 0
