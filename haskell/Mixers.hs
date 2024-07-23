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

{--
 Crazyflie QuadX:

            4     1
               x
            3     2

--}

runCF :: Demands -> (SFloat, SFloat, SFloat, SFloat)

runCF demands = (m1, m2, m3, m4) where 

   (t, r, p, y) = getDemands demands

   m1 = t - r - p  + y
   m2 = t - r + p  - y
   m3 = t + r + p  + y
   m4 = t + r - p  - y
