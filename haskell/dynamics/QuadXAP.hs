{--
  "Un-mixer" for quadrotor X configuration with Ardupilot motor numbering
 
  Copyright (C) 2021 Simon D. Levy
 
  MIT License
 --}

{-# LANGUAGE RebindableSyntax #-}

module QuadXAP where

import Language.Copilot

import Dynamics

quadXapUnmixer :: SFloat -> Unmixer

quadXapUnmixer l = Demands 0 0 0 0
