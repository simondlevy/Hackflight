{--
  Main Hackflight algorithm

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module Hackflight where

-- See Bouabdallah et al. (2004)
data Hackflight = Hackflight {value :: Double } deriving (Show)

run :: Hackflight -> Hackflight

run h = h

