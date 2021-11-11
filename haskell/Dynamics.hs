{--
  Multirotor flight dynamics
 
  Based on:
 
    @inproceedings{DBLP:conf/icra/BouabdallahMS04,
      author    = {Samir Bouabdallah and Pierpaolo Murrieri and Roland
                   Siegwart},
      title     = {Design and Control of an Indoor Micro Quadrotor},
      booktitle = {Proceedings of the 2004 {IEEE} International Conference on
                   Robotics and Automation, {ICRA} 2004, April 26 - May 1,
                   2004, New Orleans, LA, {USA}},
      pages     = {4393--4398},
      year      = {2004},
      crossref  = {DBLP:conf/icra/2004},
      url       = {https://doi.org/10.1109/ROBOT.2004.1302409},
      doi       = {10.1109/ROBOT.2004.1302409},
      timestamp = {Sun, 04 Jun 2017 01:00:00 +0200},
      biburl    = {https://dblp.org/rec/bib/conf/icra/BouabdallahMS04},
      bibsource = {dblp computer science bibliography, https://dblp.org}
    }
 
  Copyright (C) 2021 Simon D. Levy
 
  MIT License
 --}

{-# LANGUAGE RebindableSyntax #-}

module Dynamics where

import Language.Copilot

import State
import Motors
import Utils

data WorldParams = WorldParams {  g :: SFloat   -- gravitational constant
                                , rho :: SFloat -- air density
                               }

data VehicleParams = VehicleParams { d :: SFloat -- drag coefficient [T=d*w^2]
                                   , m :: SFloat -- mass [k]
                                   , ix :: SFloat -- [kg*m^2] 
                                   , iy :: SFloat -- [kg*m^2] 
                                   , iz :: SFloat -- [kg*m^2] 
                                   , jr :: SFloat -- rotor inertial [kg*m^2] 
                                   , maxrpm :: SWord16
                                   }

data FixedPitchParams = FixedPitchParams { b :: SFloat -- thrust coefficient [F=b*w^2]
                                         , l :: SFloat -- arm length [m]
                                         }

dynamics :: WorldParams -> VehicleParams -> FixedPitchParams -> Motors -> State

dynamics wparams vparams fpparams motors 
   = State x dx y dy z dz phi dphi theta dtheta psi dpsi where

  dt = stream_time - time'

  -- Convert fractional motor speed to radians per second
  rps m = (m motors) * pi / 30
  omegas_m1 = rps m1
  omegas_m2 = rps m2
  omegas_m3 = rps m3
  omegas_m4 = rps m4

  -- Thrust is squared rad/sec scaled by air density
  thrust omega = (rho wparams) * omega **2
  omegas2_m1 = thrust omegas_m1
  omegas2_m2 = thrust omegas_m2
  omegas2_m3 = thrust omegas_m3
  omegas2_m4 = thrust omegas_m4

  -- Thrust coefficient is constant for fixed-pitch rotors, variable for collective-pitch
  u1 = (b fpparams)  * (omegas2_m1 + omegas2_m2 + omegas2_m3 + omegas2_m4)

  -- Newton's third law (action/reaction) tells us that yaw is opposite to net rotor spin
  --u4 = (d vparams) * (omegas2[i] * -getrotordirection(i))
  -- omega += omegas[i] * -getRotorDirection(i);

  x = 0
  dx = if stream_time > 0 then stream_stateDx else stream_stateDx -- XXX to force stream_time for now
  y = 0
  dy = stream_stateDy
  z = 0
  dz = stream_stateDz
  phi = stream_statePhi
  dphi = stream_stateDphi
  theta = stream_stateTheta
  dtheta = stream_stateDtheta
  psi = stream_statePsi
  dpsi = stream_stateDpsi

  time' = [0] ++ stream_time

stream_time :: SFloat
stream_time = extern "stream_time" Nothing

stream_stateDx :: SFloat
stream_stateDx = extern "stream_stateDx" Nothing

stream_stateDy :: SFloat
stream_stateDy = extern "stream_stateDy" Nothing

stream_stateDz :: SFloat
stream_stateDz = extern "stream_stateDz" Nothing

stream_statePhi :: SFloat
stream_statePhi = extern "stream_statePhi" Nothing

stream_stateDphi :: SFloat
stream_stateDphi = extern "stream_stateDphi" Nothing

stream_stateTheta :: SFloat
stream_stateTheta = extern "stream_stateTheta" Nothing

stream_stateDtheta :: SFloat
stream_stateDtheta = extern "stream_stateDtheta" Nothing

stream_statePsi :: SFloat
stream_statePsi = extern "stream_statePsi" Nothing

stream_stateDpsi :: SFloat
stream_stateDpsi = extern "stream_stateDpsi" Nothing
