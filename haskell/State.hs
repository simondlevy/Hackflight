{--
  Vehicle state datatype for real and simulated flight controllers

  From Eqn. (11) in Bouabdallah,  Murrieri, Siegwart (2004). 
  We use ENU coordinates based on 
  https://www.bitcraze.io/documentation/system/platform/cf2-coordinate-system
  Position in meters, velocity in meters/second, angles in degrees,
  angular velocity in degrees/second.

  Copyright (C) 2024 Simon D. Levy
 
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, in version 3.
 
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
--} 

{-# LANGUAGE DataKinds #-}
{-# LANGUAGE RebindableSyntax #-}

module State where

import Language.Copilot
import Copilot.Compile.C99

import Utils


data State = State { 
    dx     :: SFloat 
  , dy     :: SFloat 
  , zz     :: SFloat 
  , dz     :: SFloat 
  , phi    :: SFloat 
  , dphi   :: SFloat 
  , theta  :: SFloat 
  , dtheta :: SFloat 
  , psi    :: SFloat 
  , dpsi   :: SFloat 
}
