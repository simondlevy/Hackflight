{--
  "Un-mixer" for thrust vectoring
 
  Copyright (C) 2021 Simon D. Levy
 
  MIT License
 --}

{-# LANGUAGE RebindableSyntax #-}

module ThrustVector where

import Language.Copilot

import Dynamics
import Demands
import Mixers
import Motors
import Utils


thrustVectorUnmixer :: SFloat -> Unmixer

thrustVectorUnmixer maxAngle motors = Demands t r p y where

  nozzle mval = t * sin (mval * (deg2rad maxAngle))

  -- m1, m2 = rotors
  m1val = m1 motors
  m2val = m2 motors

  t = m1val + m2val
  r = nozzle $ m3 motors
  p = nozzle $ m4 motors
  y = m1val - m2val


thrustVectorRps :: RpsFun

thrustVectorRps maxrpm (Quad m1 m2 m3 m4) = Quad rps1 rps2 m3 m4 where

  rps1 = rps maxrpm m1
  rps2 = rps maxrpm m2


thrustVectorThrust :: ThrustFun

thrustVectorThrust rho (Quad rps1 rps2 m3 m4) = Quad thrust1 thrust2 m3 m4 where

  thrust1 = thrust rps1 rho
  thrust2 = thrust rps2 rho


thrustVectorTorque :: TorqueFun

thrustVectorTorque motors = (m1 motors) - (m2 motors)

thrustVectorDynamics :: SFloat -> FrameDynamics

thrustVectorDynamics maxAngle = FrameDynamics ThrustVector
                                              (thrustVectorUnmixer maxAngle)
                                              thrustVectorRps
                                              thrustVectorThrust
                                              thrustVectorTorque
