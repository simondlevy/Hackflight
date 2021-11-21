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


quadXapRps :: RpsFun

quadXapRps maxrpm (Quad m1 m2 m3 m4) = Quad rps1 rps2 rps3 rps4 where

  rps1 = rps maxrpm m1
  rps2 = rps maxrpm m2
  rps3 = rps maxrpm m3
  rps4 = rps maxrpm m4


quadXapThrust :: ThrustFun

quadXapThrust rho (Quad rps1 rps2 rps3 rps4) = Quad thrust1 thrust2 thrust3 thrust4 where

  thrust1 = thrust rps1 rho
  thrust2 = thrust rps2 rho
  thrust3 = thrust rps3 rho
  thrust4 = thrust rps4 rho
