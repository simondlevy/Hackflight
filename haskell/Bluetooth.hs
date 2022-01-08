{--
  Hackflight Bluetooth support

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}
{-# LANGUAGE DataKinds        #-}

module Bluetooth where

import Language.Copilot

import Utils

bluetoothAvailable :: SBool
bluetoothAvailable = extern "bluetoothAvailable" Nothing

bluetoothByte :: SWord8
bluetoothByte = extern "bluetoothByte" Nothing
