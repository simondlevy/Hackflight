{--
  Utilities

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module Utils
where

-- https://stackoverflow.com/a/4343542/6110522
compose :: Foldable t => t (b -> b) -> b -> b
compose = foldr (.) id

constrain_abs :: Double -> Double -> Double
constrain_abs v lim = if v < -lim then -lim else (if v > lim then lim else v)

in_band :: Double -> Double -> Bool
in_band value band = abs(value) < band

deg2rad :: Double -> Double
deg2rad d = d * pi / 180

constrain :: Double -> Double
constrain x = if x < 0 then 0 else if x > 1 then 1 else x
