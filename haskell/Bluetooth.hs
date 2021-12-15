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

stream_bluetoothAvailable :: SBool
stream_bluetoothAvailable = extern "stream_bluetoothAvailable" Nothing

stream_bluetoothByte :: SWord8
stream_bluetoothByte = extern "stream_bluetoothByte" Nothing
