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
import Prelude hiding((<), (>), (<=), (>=), (++), (&&), not)

import State
import Mixers
import Motors
import Utils

----------------------------------------------------------------------------------------

data WorldParams = WorldParams {  g :: SFloat   -- gravitational constant
                                , rho :: SFloat -- air density
                               }

data VehicleParams = VehicleParams { d :: SFloat -- drag coefficient [T=d*w^2]
                                   , m :: SFloat -- mass [k]
                                   , ix :: SFloat -- [kg*m^2] 
                                   , iy :: SFloat -- [kg*m^2] 
                                   , iz :: SFloat -- [kg*m^2] 
                                   , jr :: SFloat -- rotor inertial [kg*m^2] 
                                   , maxrpm :: SFloat
                                   }

data FixedPitchParams = FixedPitchParams { b :: SFloat -- thrust coefficient [F=b*w^2]
                                         , l :: SFloat -- arm length [m]
                                         }

dynamics ::    WorldParams
            -> VehicleParams
            -> FixedPitchParams
            -> Mixer
            -> Motors
            -> SFloat
            -> SFloat
            -> (State, SBool)

dynamics wparams vparams fpparams mixer motors time agl
   = (State x dx y dy z dz phi dphi theta dtheta psi dpsi, airborne') where

  -- Parameter abbreviations
  b'      = b fpparams
  l'      = l fpparams
  d'      = d vparams
  m'      = m vparams
  ix'     = ix vparams
  iy'     = iy vparams
  iz'     = iz vparams
  jr'     = jr vparams
  maxrpm' = maxrpm vparams
  g'      = g wparams
  rho'    = rho wparams

  -- Compute deltaT
  dt = if time' > 0 then time - time' else 0

  -- Convert fractional motor speed to radians per second
  rps m = (m motors) * maxrpm' * pi / 30
  omegas_m1 = rps m1
  omegas_m2 = rps m2
  omegas_m3 = rps m3
  omegas_m4 = rps m4

  -- Thrust is squared rad/sec scaled by air density
  thrust o = rho' * o**2
  omegas2_m1 = thrust omegas_m1
  omegas2_m2 = thrust omegas_m2
  omegas2_m3 = thrust omegas_m3
  omegas2_m4 = thrust omegas_m4

  -- Equation 6

  u1 = b' * (omegas2_m1 + omegas2_m2 + omegas2_m3 + omegas2_m4)

  u2 = l' * b' * (-(omegas2_m1) + omegas2_m2 + omegas2_m3 - omegas2_m4)

  u3 = l' * b' * (-(omegas2_m1) + omegas2_m2 - omegas2_m3 + omegas2_m4)

  u4 = d' * (omegas2_m1 + omegas2_m2 - omegas2_m3 - omegas2_m4)

  omega = omegas2_m1 + omegas2_m2 - omegas2_m3 - omegas2_m4

  -- Use the current Euler angles to rotate the orthogonal thrust vector into the 
  -- inertial frame.  Negate to use NED.
  (accelNedX, accelNedY, accelNedZ) = bodyZToInertial ((-u1)/m') phi' theta' psi'

  -- We're airborne once net downward acceleration goes below zero
  netz = accelNedZ + g'

  lowagl = airborne' && agl <= 0 && netz >= 0

  airborne = if not airborne' && netz < 0 then true
             else if airborne' && agl <= 0 && netz >= 0 then false 
             else airborne'

  x = 0
  dx = 0

  y = 0
  dy = 0

  z   = z' + (if lowagl then agl else 0) + dt * (if airborne' then dz' else 5 * agl)
  dz  = if lowagl then 0 else dz' + (if airborne' then dt * netz else 0)

  phi    = if lowagl then 0 else phi' + (if airborne' then dt * dphi' else 0)
  dphi   = if lowagl then 0 else dphi' + (if airborne then dt * (dtheta' * dpsi' * (iy' - iz') / ix' - jr' / ix' * dtheta' * omega + u2 / ix') else 0)

  theta    = if lowagl then 0 else theta' + (if airborne' then dt * dtheta' else 0)
  dtheta   = if lowagl then 0 else dtheta' + (if airborne then dt * (dpsi' * dpsi' *(iz' - ix') / iy' + jr' / iy' * dphi' * omega + u3 / iy') else 0)

  psi = psi' + (if airborne' then dt * dpsi' else 0)
  dpsi = if lowagl then 0 else dpsi' + (if airborne' then dt * (dtheta' * dphi' * (ix' - iy') / iz' + u4 / iz') else 0) 

  x'      = [0] ++ x
  dx'     = [0] ++ dx
  y'      = [0] ++ y
  dy'     = [0] ++ dy
  z'      = [0] ++ z
  dz'     = [0] ++ dz
  phi'    = [0] ++ phi
  dphi'   = [0] ++ dphi
  theta'  = [0] ++ theta
  dtheta' = [0] ++ dtheta
  psi'    = [0] ++ psi
  dpsi'   = [0] ++ dpsi

  time' = [0] ++ time
  airborne' = [False] ++ airborne

-----------------------------------------------------------------------

  bodyZToInertial bodyZ phi theta psi = (x, y, z) where

    cph = cos phi
    sph = sin phi
    cth = cos theta
    sth = sin theta
    cps = cos psi
    sps = sin psi

    -- This is the rightmost column of the body-to-inertial rotation matrix
    x = bodyZ * (sph * sps + cph * cps * sth)
    y = bodyZ * (cph * sps * sth - cps * sph)
    z = bodyZ * (cph * cth)
