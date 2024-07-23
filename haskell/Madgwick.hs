{--
  Haskell Copilot implementation of 6DOF Madgwick algorithm
 
  Copyright (C) 2024 Simon D. Levy
 
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, in version 3.
 
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http:--www.gnu.org/licenses/>.
--} 

{-# LANGUAGE DataKinds #-}
{-# LANGUAGE RebindableSyntax #-}

module Madgwick where

import Language.Copilot hiding(atan2)
import Copilot.Compile.C99

import Prelude hiding(atan2, (++), (&&), not, (==))
import Utils

invSqrt :: SFloat -> SFloat
invSqrt x = 1 / (sqrt x)

madgwick6DOF :: (SFloat, SFloat, SFloat) -> (SFloat, SFloat, SFloat) -> SFloat ->
  (SFloat, SFloat, SFloat, SFloat)

madgwick6DOF (gx, gy, gz) (ax, ay, az) invSampFreq = (q0, q1, q2, q3) where

  -- Tunable parameter
  b_madgwick = 0.04 :: SFloat

  -- Convert gyroscope degrees/sec to radians/sec
  ggx = gx * 0.0174533
  ggy = gy * 0.0174533
  ggz = gz * 0.0174533

  -- Normalize accelerometer measurement
  recipNorm = invSqrt $ ax * ax + ay * ay + az * az
  aax = ax * recipNorm
  aay = ay * recipNorm
  aaz = az * recipNorm

  -- Rate of change of quaternion from gyroscope
  qDot1 = 0.5 * ((-q1') * ggx - q2' * ggy - q3' * ggz)
  qDot2 = 0.5 * (q0' * ggx + q2' * ggz - q3' * ggy)
  qDot3 = 0.5 * (q0' * ggy - q1' * ggz + q3' * ggx)
  qDot4 = 0.5 * (q0' * ggz + q1' * ggy - q2' * ggx)

  --Auxiliary variables to avoid repeated arithmetic
  _2q0 = 2 * q0'
  _2q1 = 2 * q1'
  _2q2 = 2 * q2'
  _2q3 = 2 * q3'
  _4q0 = 4 * q0'
  _4q1 = 4 * q1'
  _4q2 = 4 * q2'
  _8q1 = 8 * q1'
  _8q2 = 8 * q2'
  q0q0 = q0' * q0'
  q1q1 = q1' * q1'
  q2q2 = q2' * q2'
  q3q3 = q3' * q3'

  -- Gradient decent algorithm corrective step
  s0 = _4q0 * q2q2 + _2q2 * aax + _4q0 * q1q1 - _2q1 * aay
  s1 = _4q1 * q3q3 - _2q3 * aax + 4 * q0q0 * q1' - _2q0 * aay - _4q1 +
      _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * aaz 
  s2 = 4 * q0q0 * q2' + _2q0
      * aax + _4q2 * q3q3 - _2q3 * aay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 +
      _4q2 * aaz
  s3 = 4 * q1q1 * q3' - _2q1 * aax + 4 * q2q2 * q3' - _2q2 * aay

  recipNorm1 = invSqrt $ s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3

  isAccelOkay = not (ax == 0 && ay == 0 && az == 0)

  -- Compute feedback only if accelerometer measurement valid (avoids NaN in
  -- accelerometer normalisation)
  qqDot1 = qDot1 - (if isAccelOkay then b_madgwick * s0 * recipNorm1 else 0)
  qqDot2 = qDot2 - (if isAccelOkay then b_madgwick * s1 * recipNorm1 else 0)
  qqDot3 = qDot3 - (if isAccelOkay then b_madgwick * s2 * recipNorm1 else 0)
  qqDot4 = qDot4 - (if isAccelOkay then b_madgwick * s3 * recipNorm1 else 0)


  q0 = q0' + qqDot1 * invSampFreq
  q1 = q1' + qqDot2 * invSampFreq
  q2 = q2' + qqDot3 * invSampFreq
  q3 = q3' + qqDot4 * invSampFreq

  q0' = [1] ++ q0
  q1' = [0] ++ q1
  q2' = [0] ++ q2
  q3' = [0] ++ q3
