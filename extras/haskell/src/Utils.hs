{--
  Utilities

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module Utils
where

import Control.Applicative
import Data.ByteString.Internal
import Data.Either.Utils -- from MissingH
import Data.Serialize -- from cereal

constrain_abs :: Double -> Double -> Double
constrain_abs v lim = if v < -lim then -lim else (if v > lim then lim else v)

in_band :: Double -> Double -> Bool
in_band value band = abs(value) < band

deg2rad :: Double -> Double
deg2rad d = d * pi / 180

constrain :: Double -> Double
constrain x = if x < 0 then 0 else if x > 1 then 1 else x

-- https://stackoverflow.com/a/47157839
compose :: Foldable t => t1 -> t (t1 -> t1) -> t1
compose fs x = foldr (\f a -> f a) fs x

-- https://stackoverflow.com/a/20918430

doublesToBytes :: [Double] -> ByteString
doublesToBytes = runPut . mapM_ putFloat64le

bytesToDoubles :: ByteString -> [Double]
bytesToDoubles bs = (fromRight ((runGet $ many getFloat64le) bs))
