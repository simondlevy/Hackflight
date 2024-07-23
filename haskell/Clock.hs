{--
  Clock rates for real and simulated flight controllers
 
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

module Clock where

import Language.Copilot
import Copilot.Compile.C99

import Utils

data ClockRate = 
              RATE_25_HZ
            | RATE_30_HZ
            | RATE_33_HZ
            | RATE_50_HZ
            | RATE_100_HZ
            | RATE_250_HZ
            | RATE_500_HZ
            | RATE_1000_HZ

rateToPeriod :: ClockRate -> SFloat
rateToPeriod rate = 1 / (rateToFloat rate)

rateToFloat :: ClockRate -> SFloat
rateToFloat RATE_25_HZ   = 25
rateToFloat RATE_30_HZ   = 30
rateToFloat RATE_33_HZ   = 33
rateToFloat RATE_50_HZ   = 50
rateToFloat RATE_100_HZ  = 100
rateToFloat RATE_250_HZ  = 250
rateToFloat RATE_500_HZ  = 500
rateToFloat RATE_1000_HZ = 1000
