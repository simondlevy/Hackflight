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

import Utils(rad2deg)

data State = State {   x :: Stream Float
                ,     dx :: Stream Float
                ,      y :: Stream Float
                ,     dy :: Stream Float
                ,      z :: Stream Float
                ,     dz :: Stream Float
                ,    phi :: Stream Float
                ,   dphi :: Stream Float
                ,  theta :: Stream Float
                , dtheta :: Stream Float
                ,    psi :: Stream Float
                ,   dpsi :: Stream Float

              } deriving (Show)

zeroState :: State

zeroState = State 0 0 0 0 0 0 0 0 0 0 0 0


safeToArm :: State -> Stream Bool

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
