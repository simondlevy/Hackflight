{--
  Demands (thrust/roll/pitch/yaw) datatype for real and simulated flight 
  controllers
 
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

module Demands where

import Language.Copilot
import Copilot.Compile.C99

import Utils

data Demands = Demands {

     thrust :: SFloat
   , roll   :: SFloat
   , pitch  :: SFloat
   , yaw    :: SFloat
}

getDemands :: Demands -> 
                (SFloat, SFloat, SFloat, SFloat)

getDemands demands = (thrust demands, roll demands, pitch demands, yaw demands)
