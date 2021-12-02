{--
  Hackflight proxy algorithm (everything but motors)

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module HackflightProxy where

import Language.Copilot

import Receiver
import State
import Sensor
import Demands
import PidController
import Safety
import Time
import Parser
import Serial
import Utils


hackflight :: Receiver -> [Sensor] -> [PidFun] -> (Demands, MessageBuffer, SBool)

hackflight receiver sensors pidfuns = (pdemands, msgbuff, led)

  where

    -- Get receiver demands from external C functions
    rdemands = getDemands receiver

    -- Get the vehicle state by composing the sensor functions over the previous state
    state = compose sensors (state' state)

    -- Check safety (arming / failsafe)
    (armed, failsafe, cut) = safety rdemands state

    -- Periodically get the demands by composing the PID controllers over the receiver
    -- demands
    (_, _, pdemands) = compose pidfuns (state, timerReady 300, rdemands)

    -- Blink LED during first couple of seconds; keep it solid when armed
    led = if micros < 2000000 then (mod (div micros 50000) 2 == 0) else armed

    -- Run the serial comms parser
    (msgtype, sending, payindex, _checked) = parse stream_serialAvailable stream_serialByte

    -- Convert the message into a buffer to send to GCS
    msgbuff = message sending msgtype state
