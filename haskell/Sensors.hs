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

module Sensors where

import Language.Copilot
import Copilot.Compile.C99

import Utils

data ThreeAxisSensorStruct = ThreeAxisSensorStruct { 
    x'     :: Field "x" Float 
  , y'     :: Field "y" Float 
  , z'      :: Field "z" Float 
}

data ThreeAxisSensor = ThreeAxisSensor { 
    x     :: SFloat 
  , y     :: SFloat 
  , z      :: SFloat 
}

instance Struct ThreeAxisSensorStruct where

    typeName _ = "axis3" -- Name of the type in C

    toValues v = [ Value Float (x' v)
                 , Value Float (y' v)
                 , Value Float (z' v)
                 ]

instance Typed ThreeAxisSensorStruct where

  typeOf = Struct (ThreeAxisSensorStruct
                   (Field 0) 
                   (Field 0)
                   (Field 0)
                  )

liftThreeAxisSensor :: Stream ThreeAxisSensorStruct -> ThreeAxisSensor
liftThreeAxisSensor state = ThreeAxisSensor (state # x') 
                                            (state # y') 
                                            (state # z') 
