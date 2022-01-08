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

serialAvailable :: SBool
serialAvailable = extern "serialAvailable" Nothing

serialByte :: SWord8
serialByte = extern "serialByte" Nothing

serial1Available :: SBool
serial1Available = extern "serial1Available" Nothing

serial1Byte :: SWord8
serial1Byte = extern "serial1Byte" Nothing
