{--
  Hackflight full algorithm

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module HackflightFull where

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


hackflight :: Receiver -> [Sensor] -> [PidFun] -> Mixer -> (MessageBuffer, Motors, SBool)

hackflight receiver sensors pidfuns mixer = (msgbuff, motors, led)

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

    -- Run mixer on demands to get motor values
    motors' = (mixerfun mixer) (\m -> if cut then 0 else m) pdemands

    -- Blink LED during first couple of seconds; keep it solid when armed
    -- led = if micros < 2000000 then (mod (div micros 50000) 2 == 0) else armed
    led = if micros < 4000000 then (mod (div micros 50000) 2 == 0) else armed

    -- Run the serial comms parser
    (msgtyp, sending, payindex, _checked) = parse stream_serialAvailable stream_serialByte

    -- Convert the message into a buffer to send to GCS
    msgbuff = message sending msgtyp state

    -- Check for incoming SET_MOTOR messages from GCS
    motor_index = if msgtyp == 215 && payindex == 1 then stream_serialByte
                  else motor_index' where motor_index' = [0] ++ motor_index
    motor_percent = if msgtyp == 215 && payindex == 2 then stream_serialByte
                    else motor_percent' where motor_percent' = [0] ++ motor_percent

    -- Set motors based on arming state and whether we have GCS input
    motors = (motorfun mixer) motors' armed motor_index motor_percent
