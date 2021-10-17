{--
  Mixer type

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module Mixer where

import Language.Copilot
import Prelude hiding((>), (<))

import Demands
import Utils

data Motors = Motors { m1 :: SFloat
                     , m2 :: SFloat
                     , m3 :: SFloat
                     , m4 :: SFloat }

data Spins = Spins { s1 :: Demands
                   , s2 :: Demands
                   , s3 :: Demands
                   , s4 :: Demands }

mix :: Demands -> Spins -> Motors

mix demands spins = Motors m1 m2 m3 m4 where

  -- Map throttle demand from [-1,+1] to [0,1]
  t = ((throttle demands) + 1) / 2
  
  r = roll demands
  p = pitch demands
  y = yaw demands

  m1' = motor s1
  m2' = motor s2
  m3' = motor s3
  m4' = motor s4

  maxmotor = fmax (fmax (fmax m1' m2') m3') m4'

  -- This is a way to still have good gyro corrections if at least one motor reaches
  -- its max
  m1 = constrain $ cap m1'
  m2 = constrain $ cap m2'
  m3 = constrain $ cap m3'
  m4 = constrain $ cap m4'

  motor spin = let s = spin spins in
    t * (throttle s)+ r * (roll s) + p * (pitch s) + y * (yaw s)

  cap m = if m > maxmotor then m - maxmotor + 1 else m

  constrain m = if m < 0 then 0 else if m > 1 then 1 else m


p = 1 :: SFloat 
n = -1 :: SFloat

quadXMWSpins :: Spins 

quadXMWSpins = Spins (Demands p n p n)
                     (Demands p n n p)
                     (Demands p p p p)
                     (Demands p p n n)

quadXAPSpins :: Spins 

quadXAPSpins = Spins (Demands p n n p)
                     (Demands p p p p)
                     (Demands p p n n)
                     (Demands p n p n)

type Mixer = Demands -> Motors

quadXMWMixer :: Mixer

quadXMWMixer demands = mix demands quadXMWSpins 
