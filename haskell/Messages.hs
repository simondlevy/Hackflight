{--
  Code for specific MSP messages

  See https://www.hamishmb.com/multiwii/wiki/index.php?title=Multiwii_Serial_Protocol

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module Messages where

import Language.Copilot
import Copilot.Compile.C99
import Prelude hiding((==), (&&), (++))

import MSP
import Receiver
import State
import Utils

payload :: SWord8 -> State -> (SWord8, SFloat, SFloat, SFloat, SFloat, SFloat, SFloat)

payload msgtype vstate = (paysize, val00, val01, val02, val03, val04, val05) where

  paysize = if msgtype == 121 then 6 else if msgtype == 122 then 3 else 0 :: SWord8

  val00 = if msgtype == 121 then c_receiverThrottle
          else if msgtype == 122 then (phi vstate)
          else 0

  val01 = if msgtype == 121 then c_receiverRoll
          else if msgtype == 122 then (theta vstate)
          else 0

  val02 = if msgtype == 121 then c_receiverPitch
          else if msgtype == 122 then (psi vstate)
          else 0

  val03 = if msgtype == 121 then c_receiverYaw else 0
  val04 = if msgtype == 121 then c_receiverAux1 else 0
  val05 = if msgtype == 121 then c_receiverAux2 else 0

getMotors :: SWord8 -> SWord8 -> SWord8 -> (SWord8, SWord8)

getMotors msgtype payindex byte = (motor_index, motor_percent) where

  motor_index = if msgtype == 215 && payindex == 1 then byte
                else motor_index' where motor_index' = [0] ++ motor_index

  motor_percent = if msgtype == 215 && payindex == 2 then byte
                  else motor_percent' where motor_percent' = [0] ++ motor_percent

rxmessage :: SFloat -> SFloat -> SFloat -> SFloat -> SFloat -> SFloat -> Message
rxmessage thr rol pit yaw aux1 aux2 = mkcommand 0 213 0 0 thr rol pit yaw aux1 aux2

