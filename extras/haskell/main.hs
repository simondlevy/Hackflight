{--
  Copyright(C) 2021 Simon D.Levy

  MIT License
--}

import Hackflight
import Board
import OpenLoopController()

main :: IO ()

main = let h = Hackflight Board
       in putStrLn (show (run h))
