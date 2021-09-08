{--
  Utilities

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module Utils

where

import Language.Copilot hiding(xor)
import Prelude hiding ((>), (<), (&&), (==), (>>), div, mod, not)

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

data Byte = Byte {  b7 :: Stream Bool
                  , b6 :: Stream Bool
                  , b5 :: Stream Bool
                  , b4 :: Stream Bool
                  , b3 :: Stream Bool
                  , b2 :: Stream Bool
                  , b1 :: Stream Bool
                  , b0 :: Stream Bool }

word8_to_byte :: Stream Word8 -> Byte
word8_to_byte n = Byte (mod (div n 128) 2 == 0)
                       (mod (div n 64) 2 == 0)
                       (mod (div n 32) 2 == 0)
                       (mod (div n 16) 2 == 0)
                       (mod (div n 8) 2 == 0)
                       (mod (div n 4) 2 == 0)
                       (mod (div n 2) 2 == 0)
                       (mod n 2 == 0)

byte_to_word8 :: Byte -> Stream Word8
byte_to_word8 _ = 0

bit_xor :: Stream Bool -> Stream Bool -> Stream Bool
bit_xor a b = if a then not b else b

byte_xor :: Byte -> Byte -> Byte
byte_xor a b = Byte (bit_xor (b7 a) (b7 b))
                    (bit_xor (b6 a) (b6 b))
                    (bit_xor (b5 a) (b5 b))
                    (bit_xor (b4 a) (b4 b))
                    (bit_xor (b3 a) (b3 b))
                    (bit_xor (b2 a) (b2 b))
                    (bit_xor (b1 a) (b1 b))
                    (bit_xor (b0 a) (b0 b))

xor :: Stream Word8 -> Stream Word8 -> Stream Word8
xor _a b = 0
