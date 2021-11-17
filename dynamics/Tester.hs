module Tester where

import Language.Copilot
import Copilot.Compile.C99

import Dynamics


spec = do

    trigger "stream_run" true []

main = reify spec >>= compile "haskell"
