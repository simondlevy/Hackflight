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

data Message =  Message { direction :: SWord8 -- '>' (0x3E) or '<' (0x3C)
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

  val00 = if msgtype == 105 then unsafeCast c_receiverThrottle
          else if msgtype == 108 then 10 * (rad2deg (phi vstate))
          else 0

  val01 = if msgtype == 105 then unsafeCast c_receiverRoll
          else if msgtype == 108 then 10 * (rad2deg (theta vstate))
          else 0

  val02 = if msgtype == 105 then unsafeCast c_receiverPitch
          else if msgtype == 108 then rad2deg (psi vstate)
          else 0

  val03 = if msgtype == 105 then unsafeCast c_receiverYaw else 0
  val04 = if msgtype == 105 then unsafeCast c_receiverAux1 else 0
  val05 = if msgtype == 105 then unsafeCast c_receiverAux2 else 0

----------------------------------------------------------------------------

rxmessage :: SFloat -> SFloat -> SFloat -> SFloat -> SFloat -> SFloat -> Message

rxmessage c1 c2 c3 c4 c5 c6 = Message 0x3E 12 200 c1 c2 c3 c4 c5 c6 where

----------------------------------------------------------------------------

makeval :: SWord8 -> SWord8 -> SWord8 -> (SWord16, SBool)

makeval byte payindex index = (value, ready) where

  ready = index == div payindex 2

  b = cast byte :: SWord16
  b' = [0] ++ b
  value = if mod payindex 2 == 0 then b' .|. b.<<.(8::SWord8)
           else  value' where value' = [0] ++ value :: SWord16

----------------------------------------------------------------------------

getMotors :: SWord8 -> SWord8 -> SWord8 -> (SFloat, SFloat, SFloat, SFloat)

getMotors msgtype payindex byte = (m1, m2, m3, m4) where

  m1 = getmval 1 m1' where m1' = [0] ++ m1
  m2 = getmval 2 m2' where m2' = [0] ++ m2
  m3 = getmval 3 m3' where m3' = [0] ++ m3
  m4 = getmval 4 m4' where m4' = [0] ++ m4

  getmval index oldval = let (value, ready) = makeval byte payindex index
                         in if ready then ((unsafeCast value) - 1000) / 1000
                            else oldval :: SFloat

----------------------------------------------------------------------------

getChannels :: SWord8 -> SWord8 -> SWord8
                -> (SWord16, SWord16, SWord16, SWord16, SWord16, SWord16)

getChannels msgtype payindex byte = (c1, c2, c3, c4, c5, c6) where

  c1 = getcval 1 c1' where c1' = [0] ++ c1
  c2 = getcval 2 c2' where c2' = [0] ++ c2
  c3 = getcval 3 c3' where c3' = [0] ++ c3
  c4 = getcval 4 c4' where c4' = [0] ++ c4
  c5 = getcval 5 c5' where c5' = [0] ++ c5
  c6 = getcval 6 c6' where c6' = [0] ++ c6

  getcval index oldval = let (value, ready) = makeval byte payindex index
                         in if ready then value else oldval


