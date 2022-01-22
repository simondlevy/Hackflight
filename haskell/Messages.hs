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
                        , f1        :: SFloat
                        , f2        :: SFloat
                        , f3        :: SFloat
                        , f4        :: SFloat
                        , f5        :: SFloat
                        , f6        :: SFloat
                        , b01       :: SWord8
                        , b02       :: SWord8
                        , b03       :: SWord8
                        , b04       :: SWord8
                        , b05       :: SWord8
                        , b06       :: SWord8
                        , b07       :: SWord8
                        , b08       :: SWord8
                        , b09       :: SWord8
                        , b10       :: SWord8
                        , b11       :: SWord8
                        , b12       :: SWord8
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

rxmessage :: SFloat -> SFloat -> SFloat -> SFloat -> SFloat -> SFloat -> Message

rxmessage c1 c2 c3 c4 c5 c6 = (Message 0x3E 12 200 v1 v2 v3 v4 v5 v6
                                       0 0 0 0 0 0 0 0 0 0 0 0) where
  v1 = scale c1
  v2 = scale c2
  v3 = scale c3
  v4 = scale c4
  v5 = scale c5
  v6 = scale c6

  scale c = 1000 * (1 + (c + 1) / 2)

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


