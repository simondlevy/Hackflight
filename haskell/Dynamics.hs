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

data WorldParams = WorldParams {  g :: SDouble   -- gravitational constant
                                , rho :: SDouble -- air density
                               }

data VehicleParams = VehicleParams { d :: SDouble -- drag coefficient [T=d*w^2]
                                   , m :: SDouble -- mass [k]
                                   , ix -- [kg*m^2] 
                                   , iy -- [kg*m^2] 
                                   , iz -- [kg*m^2] 
                                   , jr -- rotor inertial [kg*m^2] 
                                   , maxrpm :: SWord16
                                   }

dynamics :: SDouble -> Motors -> State

dynamics time _motors = State x dx y dy z dz phi dphi theta dtheta psi dpsi where
  dt = time - time'

  x = 0
  dx = if time > 0 then stream_stateDx else stream_stateDx
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

  time' = [0] ++ time

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
