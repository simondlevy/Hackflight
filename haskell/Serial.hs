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

c_serialAvailable :: SBool
c_serialAvailable = extern "serialAvailable" Nothing

c_serialByte :: SWord8
c_serialByte = extern "serialByte" Nothing

c_serial1Available :: SBool
c_serial1Available = extern "serial1Available" Nothing

c_serial1Byte :: SWord8
c_serial1Byte = extern "serial1Byte" Nothing
