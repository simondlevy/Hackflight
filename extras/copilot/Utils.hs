{--
  Utilities

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module Utils

where

import Language.Copilot
import Prelude hiding ((>), (<), (&&), div)

-- https://stackoverflow.com/a/4343542/6110522
compose :: Foldable t => t (b -> b) -> b -> b
compose = foldr (.) id

constrain_abs :: Stream Double -> Stream Double -> Stream Double
constrain_abs v lim = if v < (-lim) then (-lim) else (if v > lim then lim else v)

in_band :: Stream Double -> Stream Double -> Stream Bool
in_band value band = value > (-band) && value < band

deg2rad :: Stream Double -> Stream Double
deg2rad d = d * pi / 180

constrain :: Stream Double -> Stream Double
constrain x = if x < 0 then 0 else if x > 1 then 1 else x
