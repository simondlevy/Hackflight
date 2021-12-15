{--
  Hackflight serial comms support

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}
{-# LANGUAGE DataKinds        #-}

module Serial where

import Language.Copilot

import Utils

stream_serialAvailable :: SBool
stream_serialAvailable = extern "stream_serialAvailable" Nothing

stream_serialByte :: SWord8
stream_serialByte = extern "stream_serialByte" Nothing

stream_serial1Available :: SBool
stream_serial1Available = extern "stream_serial1Available" Nothing

stream_serial1Byte :: SWord8
stream_serial1Byte = extern "stream_serial1Byte" Nothing
