{--
  Utilities

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module Utils
where

constrain_abs :: Double -> Double -> Double
constrain_abs v lim = if v < -lim then -lim else (if v > lim then lim else v)

in_band :: Double -> Double -> Bool
in_band value band = abs(value) < band

deg2rad :: Double -> Double
deg2rad d = d * pi / 180
