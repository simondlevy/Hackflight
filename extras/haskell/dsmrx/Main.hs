{--
  Haskell Copilot support for DSMX receivers

  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}
{-# LANGUAGE DataKinds        #-}

module Main where

import Language.Copilot
import Copilot.Compile.C99

spec = do


  -- Send the motor values to the external C function
  trigger "copilot_reportReceiver" true []

-- Compile the spec
main = reify spec >>= compile "copilot"
