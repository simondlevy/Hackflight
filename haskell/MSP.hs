{--
  General code for MSP messages

  See https://www.hamishmb.com/multiwii/wiki/index.php?title=Multiwii_Serial_Protocol

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module MSP where

import Language.Copilot
import Copilot.Compile.C99

import Utils

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

mkresponse ::  SWord8  -- size
            -> SWord8  -- type
            -> SFloat  -- v1
            -> SFloat  -- v2
            -> SFloat  -- v3
            -> SFloat  -- v4
            -> SFloat  -- v5
            -> SFloat  -- v6
            -> Message

mkresponse size msgtype v1 v2 v3 v4 v5 v6 =
  Message 0x3E size msgtype v1 v2 v3 v4 v5 v6

mkcommand ::   SWord8  -- size
            -> SWord8  -- type
            -> SFloat  -- v1
            -> SFloat  -- v2
            -> SFloat  -- v3
            -> SFloat  -- v4
            -> SFloat  -- v5
            -> SFloat  -- v6
            -> Message

mkcommand size msgtype v1 v2 v3 v4 v5 v6 =
  Message 0x3C size msgtype v1 v2 v3 v4 v5 v6

