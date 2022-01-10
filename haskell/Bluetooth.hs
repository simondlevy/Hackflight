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

c_bluetoothAvailable :: SBool
c_bluetoothAvailable = extern "bluetoothAvailable" Nothing

c_bluetoothByte :: SWord8
c_bluetoothByte = extern "bluetoothByte" Nothing
