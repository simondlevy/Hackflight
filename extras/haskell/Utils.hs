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

-- https://stackoverflow.com/questions/20912582/haskell-bytestring-to-float-array

doublesToBytes :: [Double] -> ByteString
doublesToBytes = runPut . mapM_ putFloat64le

bytesToDoubles :: ByteString -> [Double]
bytesToDoubles bs = (fromRight ((runGet $ many getFloat64le) bs))
