{--
  "Un-mixer" for coaxial vehicles

   XXX currently just faked up as QuadXAP
 
  Copyright (C) 2021 Simon D. Levy
 
  MIT License
 --}

{-# LANGUAGE RebindableSyntax #-}

module Coaxial where

import Language.Copilot

import Dynamics
import Demands
import Mixers
import Motors
import Utils


coaxialUnmixer :: Unmixer

coaxialUnmixer motors = Demands t r p y where

  t = (m1 motors) + (m2 motors) + (m3 motors) + (m4 motors)
  r = (m2 motors) + (m3 motors) - (m1 motors) - (m4 motors)
  p = (m2 motors) + (m4 motors) - (m1 motors) - (m3 motors)
  y = (m1 motors) + (m2 motors) - (m3 motors) - (m4 motors)


coaxialRps :: RpsFun

coaxialRps maxrpm (Quad m1 m2 m3 m4) = Quad rps1 rps2 rps3 rps4 where

  rps1 = rps maxrpm m1
  rps2 = rps maxrpm m2
  rps3 = rps maxrpm m3
  rps4 = rps maxrpm m4


coaxialThrust :: ThrustFun

coaxialThrust rho (Quad rps1 rps2 rps3 rps4) = Quad thrust1 thrust2 thrust3 thrust4 where

  thrust1 = thrust rps1 rho
  thrust2 = thrust rps2 rho
  thrust3 = thrust rps3 rho
  thrust4 = thrust rps4 rho


coaxialTorque :: TorqueFun

coaxialTorque motors = (m1 motors) + (m2 motors) - (m3 motors) - (m4 motors)

coaxialDynamics :: FrameDynamics

coaxialDynamics = FrameDynamics Coaxial
                                coaxialUnmixer
                                coaxialRps
                                coaxialThrust
                                coaxialTorque
