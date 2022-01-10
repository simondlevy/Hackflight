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


data Message' = Message' {  hdr0'    :: SWord8 -- '$' (0x24)
                          , hdr1'    :: SWord8 -- 'M' (0x4D)
                          , hdr2'    :: SWord8 -- '>' (0x3E) or '<' (0x3C)
                          , outsize' :: SWord8 
                          , msgtype' :: SWord8 
                          , crc'     :: SWord8 
                          , paysize' :: SWord8 
                          , b01'     :: SWord8 
                          , b02'     :: SWord8 
                          , b03'     :: SWord8 
                          , b04'     :: SWord8 
                          , b05'     :: SWord8 
                          , b06'     :: SWord8 
                          , b07'     :: SWord8 
                          , b08'     :: SWord8 
                          , b09'     :: SWord8 
                          , b10'     :: SWord8 
                          , b11'     :: SWord8 
                          , b12'     :: SWord8 
                         }

-- Use floats for every payload
data Message = Message {  hdr0    :: SWord8 -- '$' (0x24)
                        , hdr1    :: SWord8 -- 'M' (0x4D)
                        , hdr2    :: SWord8 -- '>' (0x3E) or '<' (0x3C)
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

mkmessage ::   SWord8
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
            -> Message

mkmessage direction outsize msgtype crc paysize v00 v01 v02 v03 v04 v05 =
  Message 0x24 0x4D direction outsize msgtype crc paysize v00 v01 v02 v03 v04 v05

mkresponse ::  SWord8  -- outsize
            -> SWord8  -- msgtype
            -> SWord8  -- crc
            -> SWord8  -- paysize
            -> SFloat  -- v00
            -> SFloat  -- v01
            -> SFloat  -- v02
            -> SFloat  -- v03
            -> SFloat  -- v04
            -> SFloat  -- v05
            -> Message

mkresponse outsize msgtype crc paysize v00 v01 v02 v03 v04 v05 =
  mkmessage 0x3E outsize msgtype crc paysize v00 v01 v02 v03 v04 v05

mkcommand ::   SWord8  -- outsize
            -> SWord8  -- msgtype
            -> SWord8  -- crc
            -> SWord8  -- paysize
            -> SFloat  -- v00
            -> SFloat  -- v01
            -> SFloat  -- v02
            -> SFloat  -- v03
            -> SFloat  -- v04
            -> SFloat  -- v05
            -> Message

mkcommand outsize msgtype crc paysize v00 v01 v02 v03 v04 v05 =
  mkmessage 0x3C outsize msgtype crc paysize v00 v01 v02 v03 v04 v05

