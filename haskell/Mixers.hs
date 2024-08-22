{--
  Motor-mixing functions for real and simulated flight controllers
 
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

module Mixers where

import Language.Copilot
import Copilot.Compile.C99

import Demands
import Utils

runQuadX :: Demands -> (SFloat, SFloat, SFloat, SFloat)

runQuadX demands = (nw, se, sw, ne) where 

   (t, r, p, y) = getDemands demands

   nw = t - r - p  + y
   se = t - r + p  - y
   sw = t + r + p  + y
   ne = t + r - p  - y
