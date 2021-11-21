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
import Motors
import Utils

quadXapUnmixer :: SFloat -> Unmixer

quadXapUnmixer l motors = Demands t r p y where

  t = l * ((m1 motors) + (m2 motors) + (m3 motors) + (m4 motors))
  r = l * ((m2 motors) + (m3 motors) - (m1 motors) - (m4 motors))
  p = l * ((m2 motors) + (m4 motors) - (m1 motors) - (m3 motors))
  y = l * ((m1 motors) + (m2 motors) - (m3 motors) - (m4 motors))
