{--
  Haskell Copilot support for sensors

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

module Sensor

where

import Language.Copilot

import State

type Sensor = State -> State
