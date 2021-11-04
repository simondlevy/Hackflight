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

zeroState :: State

zeroState = State 0 0 0 0 0 0 0 0 0 0 0 0


safeToArm :: State -> SBool

safeToArm (State _ _ _ _ _ _ phi _ theta _ _ _) = safe phi && safe theta
  where safe angle = (abs (rad2deg angle)) < 25


addStates :: State -> State -> State

addStates v1 v2 = 
    State ((x v1)      + (x v2))
          ((dx v1)     + (dx v2))
          ((y v1)      + (y v2))
          ((dy v1)     + (dy v2))
          ((z v1)      + (z v2))
          ((dz v1)     + (dz v2))
          ((phi v1)    + (phi v2))
          ((dphi v1)   + (dphi v2))
          ((theta v1)  + (theta v2))
          ((dtheta v1) + (dtheta v2))
          ((psi v1)    + (psi v2))
          ((dpsi v1)   + (dpsi v2))
