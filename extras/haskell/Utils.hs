{--
  Utilities

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module Utils

where

import Language.Copilot hiding(xor)
import Prelude hiding ((>), (<), (&&), div)

-- https://stackoverflow.com/a/4343542/6110522
compose :: Foldable t => t (b -> b) -> b -> b
compose = foldr (.) id

constrain_abs :: Stream Float -> Stream Float -> Stream Float
constrain_abs v lim = if v < (-lim) then (-lim) else (if v > lim then lim else v)

in_band :: Stream Float -> Stream Float -> Stream Bool
in_band value band = value > (-band) && value < band

deg2rad :: Stream Float -> Stream Float
deg2rad d = d * pi / 180

rad2deg :: Stream Float -> Stream Float
rad2deg r = r * 180 / pi

constrain :: Stream Float -> Stream Float
constrain x = if x < 0 then 0 else if x > 1 then 1 else x

data Byte = Byte {  b0 :: Stream Bool
                  , b1 :: Stream Bool
                  , b2 :: Stream Bool
                  , b3 :: Stream Bool
                  , b4 :: Stream Bool
                  , b5 :: Stream Bool
                  , b6 :: Stream Bool
                  , b7 :: Stream Bool }

xor :: Stream Word8 -> Stream Word8 -> Stream Word8
xor _a b = 0
