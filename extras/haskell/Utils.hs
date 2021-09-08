{--
  Utilities

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module Utils

where

import Language.Copilot hiding(xor)
import Prelude hiding ((>), (<), (&&), (==), (>>), (/=), div, mod, not)

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
byte_to_word8 byte =  128 * (if b7 byte then 1 else 0)
                    +  64 * (if b6 byte then 1 else 0)
                    +  32 * (if b5 byte then 1 else 0)
                    +  16 * (if b4 byte then 1 else 0)
                    +   8 * (if b3 byte then 1 else 0)
                    +   4 * (if b2 byte then 1 else 0)
                    +   2 * (if b1 byte then 1 else 0)
                    +       (if b0 byte then 1 else 0)

byte_xor :: Byte -> Byte -> Byte
byte_xor a b = Byte ((b7 a) /= (b7 b))
                    ((b6 a) /= (b6 b))
                    ((b5 a) /= (b5 b))
                    ((b4 a) /= (b4 b))
                    ((b3 a) /= (b3 b))
                    ((b2 a) /= (b2 b))
                    ((b1 a) /= (b1 b))
                    ((b0 a) /= (b0 b))

xor :: Stream Word8 -> Stream Word8 -> Stream Word8
xor a b = byte_to_word8 (byte_xor (word8_to_byte a) (word8_to_byte b))
