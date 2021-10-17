{--
  Vehicle state

  See Bouabdallah et al. (2004)

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module State where

import Language.Copilot
import Prelude hiding((&&), (<))

import Utils

data State = State {   x :: SFloat
                ,     dx :: SFloat
                ,      y :: SFloat
                ,     dy :: SFloat
                ,      z :: SFloat
                ,     dz :: SFloat
                ,    phi :: SFloat
                ,   dphi :: SFloat
                ,  theta :: SFloat
                , dtheta :: SFloat
                ,    psi :: SFloat
                ,   dpsi :: SFloat

              }

safeToArm :: State -> SBool

safeToArm (State _ _ _ _ _ _ phi _ theta _ _ _) = safe phi && safe theta
  where safe angle = (abs (rad2deg angle)) < 25
