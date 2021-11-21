{--
  "Un-mixer" for quadrotor X configuration with Ardupilot motor numbering
 
  Copyright (C) 2021 Simon D. Levy
 
  MIT License
 --}

{-# LANGUAGE RebindableSyntax #-}

module QuadXAP where

import Language.Copilot

import Dynamics
import Demands
import Utils

quadXapUnmixer :: SFloat -> Unmixer

quadXapUnmixer l motors = Demands 0 0 0 0
