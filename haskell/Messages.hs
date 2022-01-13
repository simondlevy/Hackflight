{--
  Code for MSP messages

  See https://www.hamishmb.com/multiwii/wiki/index.php?title=Multiwii_Serial_Protocol

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module Messages where

import Language.Copilot
import Copilot.Compile.C99
import Prelude hiding((==), (&&), (++), mod, div)

import Receiver
import State
import Utils

----------------------------------------------------------------------------

-- Use floats for every payload
data Message = Message {  
                          direction :: SWord8 -- '>' (0x3E) or '<' (0x3C)
                        , paysize   :: SWord8 
                        , msgtype   :: SWord8 
                        , v1        :: SFloat
                        , v2        :: SFloat
                        , v3        :: SFloat
                        , v4        :: SFloat
                        , v5        :: SFloat
                        , v6        :: SFloat
                        }

----------------------------------------------------------------------------

payload :: SWord8 -> State -> (SWord8, SFloat, SFloat, SFloat, SFloat, SFloat, SFloat)

payload msgtype vstate = (paysize, val00, val01, val02, val03, val04, val05) where

  paysize = if msgtype == 105 then 12 else if msgtype == 108 then 6 else 0 :: SWord8

  val00 = if msgtype == 105 then rxscale c_receiverThrottle
          else if msgtype == 108 then 10 * (rad2deg (phi vstate))
          else 0

  val01 = if msgtype == 105 then rxscale c_receiverRoll
          else if msgtype == 108 then 10 * (rad2deg (theta vstate))
          else 0

  val02 = if msgtype == 105 then rxscale c_receiverPitch
          else if msgtype == 108 then rad2deg (psi vstate)
          else 0

  val03 = if msgtype == 105 then rxscale c_receiverYaw else 0
  val04 = if msgtype == 105 then rxscale c_receiverAux1 else 0
  val05 = if msgtype == 105 then rxscale c_receiverAux2 else 0

  rxscale x = 1000 + 1000 * (x + 1) / 2

----------------------------------------------------------------------------

getMotors :: SWord8 -> SWord8 -> SWord8 -> (SFloat, SFloat, SFloat, SFloat)

getMotors msgtype payindex byte = (m1, m2, m3, m4) where

  mindex = div payindex 2

  -- m1 = if mindex == 1 then (unsafeCast mvalue) else m1' where m1' = [0] ++ m1
  m2 = if mindex == 2 then (unsafeCast mvalue) else m2' where m2' = [0] ++ m2
  m3 = if mindex == 3 then (unsafeCast mvalue) else m3' where m3' = [0] ++ m3
  m4 = if mindex == 4 then (unsafeCast mvalue) else m4' where m4' = [0] ++ m4

  m1 = getmval 1 m1' where m1' = [0] ++ m1
  getmval idx oldval = if mindex == idx then (unsafeCast mvalue) else oldval :: SFloat

  -- Make a 16-bit motor value from two-byte pairs
  b = cast byte :: SWord16
  b' = [0] ++ b
  mvalue = if mod payindex 2 == 0 then b' .|. b.<<.(8::SWord8)
           else  mvalue' where mvalue' = [0] ++ mvalue :: SWord16
