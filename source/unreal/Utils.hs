{--
  Utilities

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module Utils

where

import Language.Copilot hiding(xor)
import Copilot.Language.Operators.BitWise(complement, (.&.), (.|.))
import Copilot.Library.Stacks
import Prelude hiding ((>), (<), (&&), (==), (>>), (/=), div, mod, not)

type SFloat = Stream Float
type SWord8 = Stream Word8
type SWord32 = Stream Word32
type SBool = Stream Bool

-- https://stackoverflow.com/a/4343542/6110522
compose :: Foldable t => t (b -> b) -> b -> b
compose = foldr (.) id

constrain_abs :: SFloat -> SFloat -> SFloat
constrain_abs v lim = if v < (-lim) then (-lim) else (if v > lim then lim else v)

in_band :: SFloat -> SFloat -> Stream Bool
in_band value band = value > (-band) && value < band

deg2rad :: SFloat -> SFloat
deg2rad d = d * pi / 180

rad2deg :: SFloat -> SFloat
rad2deg r = r * 180 / pi

constrain :: SFloat -> SFloat
constrain x = if x < 0 then 0 else if x > 1 then 1 else x

xor :: SWord8 -> SWord8 -> SWord8
xor a b = (a .|. b) .&. (complement (a .&. b))
