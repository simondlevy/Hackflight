{--
  Motor configurations and safety features

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module Motors where

import Language.Copilot

import Utils

data Motors = Quad { m1 :: SFloat
                   , m2 :: SFloat
                   , m3 :: SFloat
                   , m4 :: SFloat }

type MotorFun = Motors -> SBool -> SFloat -> SFloat -> SFloat -> SFloat -> Motors

motorval :: SBool -> SFloat -> SFloat -> SFloat

motorval armed flyval gcsval = if armed then flyval else gcsval

quadfun :: MotorFun

quadfun motors armed m1gcs m2gcs m3gcs m4gcs = Quad m1val m2val m3val m4val where

  m1val = motorval armed (m1 motors) m1gcs
  m2val = motorval armed (m2 motors) m2gcs
  m3val = motorval armed (m3 motors) m3gcs
  m4val = motorval armed (m4 motors) m4gcs

motors' :: Motors -> Motors

motors' (Quad m1 m2 m3 m4) = Quad ([0] ++ m1) ([0] ++ m2) ([0] ++ m3) ([0] ++ m4)
