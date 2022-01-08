{--
  Code for specific MSP messages

  See https://www.hamishmb.com/multiwii/wiki/index.php?title=Multiwii_Serial_Protocol

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module Messages where

import Language.Copilot hiding(xor)
import Copilot.Compile.C99
import Prelude hiding((==), (&&), (++))

import Receiver
import State
import Utils

data MessageBuffer = MessageBuffer {  sending :: SBool
                                    , hdr0    :: SWord8 -- '$' (0x24)
                                    , hdr1    :: SWord8 -- 'M' (0x4D)
                                    , hdr2    :: SWord8 -- '>' (0x3E)
                                    , outsize :: SWord8 
                                    , msgtype :: SWord8 
                                    , crc     :: SWord8 
                                    , paysize :: SWord8 
                                    , val00   :: SFloat  
                                    , val01   :: SFloat
                                    , val02   :: SFloat
                                    , val03   :: SFloat
                                    , val04   :: SFloat
                                    , val05   :: SFloat
                                    }

mkmessage ::   SBool
            -> SWord8
            -> SWord8
            -> SWord8
            -> SWord8
            -> SFloat
            -> SFloat
            -> SFloat
            -> SFloat
            -> SFloat
            -> SFloat
            -> MessageBuffer

mkmessage sending outsize msgtype crc paysize v00 v01 v02 v03 v04 v05 =
  MessageBuffer sending 0x24 0x4D 0x3E outsize msgtype crc paysize v00 v01 v02 v03 v04 v05

payload :: SWord8 -> State -> (SWord8, SFloat, SFloat, SFloat, SFloat, SFloat, SFloat)

payload msgtype vstate = (paysize, val00, val01, val02, val03, val04, val05) where

  paysize = if msgtype == 121 then 6 else if msgtype == 122 then 3 else 0 :: SWord8

  val00 = if msgtype == 121 then receiverThrottle
          else if msgtype == 122 then (phi vstate)
          else 0

  val01 = if msgtype == 121 then receiverRoll
          else if msgtype == 122 then (theta vstate)
          else 0

  val02 = if msgtype == 121 then receiverPitch
          else if msgtype == 122 then (psi vstate)
          else 0

  val03 = if msgtype == 121 then receiverYaw else 0
  val04 = if msgtype == 121 then receiverAux1 else 0
  val05 = if msgtype == 121 then receiverAux2 else 0

getMotors :: SWord8 -> SWord8 -> SWord8 -> (SWord8, SWord8)

getMotors msgtype payindex byte = (motor_index, motor_percent) where

  motor_index = if msgtype == 215 && payindex == 1 then byte
                else motor_index' where motor_index' = [0] ++ motor_index

  motor_percent = if msgtype == 215 && payindex == 2 then byte
                  else motor_percent' where motor_percent' = [0] ++ motor_percent

rxmessage :: SFloat -> SFloat -> SFloat -> SFloat -> SFloat -> SFloat -> MessageBuffer

rxmessage thr rol pit yaw aux1 aux2 = MessageBuffer true 0 0 0 0 0 0 0 0 0 0 0 0 0

