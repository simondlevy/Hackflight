{--
  Hackflight serial parsing

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}
{-# LANGUAGE DataKinds        #-}

module Parser where

import Language.Copilot hiding(xor)
import Copilot.Language.Operators.BitWise((.|.), (.<<.))

import Utils

buffer :: Stream (Array 128 Word8)
buffer = extern "stream_buffer" Nothing

serialAvailable :: Stream Bool
serialAvailable = extern "stream_serialAvailable" Nothing

serialByteIn :: Stream Word8
serialByteIn = extern "stream_serialByte" Nothing
