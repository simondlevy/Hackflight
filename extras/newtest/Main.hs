{--
  Copyright(C) 2021 on D.Levy

  MIT License
--}

{-# LANGUAGE RebindableSyntax #-}

module Main where

import Language.Copilot
import Copilot.Compile.C99
import Prelude hiding((==), (++))

type SWord8 = Stream Word8

data Buffer = Buffer {  byte0 :: SWord8
                      , byte1 :: SWord8
                      , byte2 :: SWord8
                      , byte3 :: SWord8 }
           

setbuff :: Buffer -> SWord8 -> SWord8 -> Buffer

setbuff buffer index value = buffer'

  where buffer' = Buffer byte0 byte1 byte2 byte3

        byte0 = if index == 0 then value else byte0'
        byte1 = if index == 1 then value else byte1'
        byte2 = if index == 2 then value else byte2'
        byte3 = if index == 3 then value else byte3'

        byte0' = [0] ++ byte0
        byte1' = [0] ++ byte1
        byte2' = [0] ++ byte2
        byte3' = [0] ++ byte3

getbuff :: Buffer -> SWord8 -> SWord8

getbuff buffer index = value

  where value = if index == 0 then byte0 buffer
                else if index == 1 then byte1 buffer
                else if index == 2 then byte2 buffer
                else if index == 3 then byte3 buffer
                else 0

newbuff :: Buffer

newbuff = Buffer 0 0 0 0

spec = do

  let buff = newbuff

  let buff' = setbuff buff 1 99

  trigger "report" true [arg $ getbuff buff' 1]

-- Compile the spec
main = reify spec >>= compile "copilot"
