{--
  Vehicle state

  See Bouabdallah et al. (2004)

  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module State where

import Language.Copilot
import Prelude hiding((&&), (<), (++))

import Utils

type StateFun = State -> State

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
    

state' :: State -> State

state' state = State ([0] ++ (x state))
                     ([0] ++ (dx state))
                     ([0] ++ (y state))
                     ([0] ++ (dy state))
                     ([0] ++ (z state))
                     ([0] ++ (dz state))
                     ([0] ++ (phi state))
                     ([0] ++ (dphi state))
                     ([0] ++ (theta state))
                     ([0] ++ (dtheta state))
                     ([0] ++ (psi state))
                     ([0] ++ (dpsi state))
